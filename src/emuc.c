/*
 * emuc.c - Innodisk EMUC-B201 CAN interface driver (using tty line discipline)
 *
 * This file is derived from linux/drivers/net/slip/slip.c
 *
 * slip.c Authors  : Laurence Culhane <loz@holmes.demon.co.uk>
 *                   Fred N. van Kempen <waltje@uwalt.nl.mugnet.org>
 * slcan.c Author  : Oliver Hartkopp <socketcan@hartkopp.net>
 * emuc.c Authors  : Innodisk
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307. You can also get it
 * at http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/can.h>
#include <linux/version.h>

#include "lib_emuccan.h"

/* 
 * can_skb_prv apis avaiable at kernel 3.9,
 * reference 3.8's slcan.c, just ignore it 
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
#include <linux/can/skb.h>
#endif

/*
 * Ldisc number for emun.
 * Before kernel 2.6.35, there is no extra preserved ldisc 
 * number can be use. So, I decide to steal the number which
 * SLCAN used.
 * That means, this driver and slcan cannnot be loaded at
 * the same time on kernel before 2.6.35.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
#define N_EMUC (NR_LDISCS - 1)
#else
#define N_EMUC N_SLCAN
#endif

/* Allen 2017.02.14 queue buf issue */
#define FIXED_QUEUE_BUF 

static __initconst const char banner[] =
	KERN_INFO "emuc: EMUC-B201 CAN interface driver\n";

MODULE_ALIAS_LDISC(emuccan);
MODULE_DESCRIPTION("EMUC-B201 CAN interface driver");
MODULE_VERSION("0:1.9");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Innodisk");

#define EMUC_MAGIC 0x729B

static int maxdev = 10;		/* MAX number of EMUC channels;
				   This can be overridden with
				   insmod emuc.ko maxdev=nnn	*/
module_param(maxdev, int, 0);
MODULE_PARM_DESC(maxdev, "Maximum number of emuc interfaces");

/* maximum rx buffer len: extended CAN frame with timestamp */
#define EMUC_MTU (sizeof(":0301112233441122334455667788FF\r\n")+1)

struct emuc {
	int			magic;

	/* Various fields. */
	struct tty_struct	*tty;		/* ptr to TTY structure	     */
	struct net_device	*devs[2];	/* easy for intr handling    */
	spinlock_t		lock;
	struct work_struct	tx_work;	/* Flushes transmit buffer   */
	atomic_t		ref_count;	/* reference count           */
	int			gif_channel;	/* index for SIOCGIFNAME     */
	unsigned char current_channel;  /* Record current channel: for fixing tx_packet bug (v1.8) */


	/* These are pointers to the malloc()ed frame buffers. */
	unsigned char		rbuff[EMUC_MTU];/* receiver buffer	     */
	int			rcount;         /* received chars counter    */
	unsigned char		xbuff[EMUC_MTU];/* transmitter buffer	     */
	unsigned char		*xhead;         /* pointer to next XMIT byte */
	int			xleft;          /* bytes left in XMIT queue  */

	unsigned long		flags;		/* Flag values/ mode etc     */
#define SLF_INUSE		0		/* Channel in use            */
#define SLF_ERROR		1               /* Parity, etc. error        */
};

struct emuc_priv {
	int			magic;
	struct emuc		*info;		/* just ptr to emuc_info     */
};

static struct net_device **emuc_devs;

 /************************************************************************
  *			EMUC ENCAPSULATION FORMAT			 *
  ************************************************************************/

/*
 * A CAN frame has a can_id (11 bit standard frame format OR 29 bit extended
 * frame format) a data length code (can_dlc) which can be from 0 to 8
 * and up to <can_dlc> data bytes as payload.
 * Additionally a CAN frame may become a remote transmission frame if the
 * RTR-bit is set. This causes another ECU to send a CAN frame with the
 * given can_id.
 *
 * The EMUC ASCII representation of these different frame types is:
 * <type> <id> <dlc> <data>*
 *
 * Extended frames (29 bit) are defined by capital characters in the type.
 * RTR frames are defined as 'r' types - normal frames have 't' type:
 * t => 11 bit data frame
 * r => 11 bit RTR frame
 * T => 29 bit data frame
 * R => 29 bit RTR frame
 *
 * The <id> is 3 (standard) or 8 (extended) bytes in ASCII Hex (base64).
 * The <dlc> is a one byte ASCII number ('0' - '8')
 * The <data> section has at much ASCII Hex bytes as defined by the <dlc>
 *
 * Examples:
 *
 * t1230 : can_id 0x123, can_dlc 0, no data
 * t4563112233 : can_id 0x456, can_dlc 3, data 0x11 0x22 0x33
 * T12ABCDEF2AA55 : extended can_id 0x12ABCDEF, can_dlc 2, data 0xAA 0x55
 * r1230 : can_id 0x123, can_dlc 0, no data, remote transmission request
 *
 */

 /************************************************************************
  *			STANDARD EMUC DECAPSULATION			 *
  ************************************************************************/

/* Send one completely decapsulated can_frame to the network layer */
static void emuc_bump(struct emuc *info)
{
	EMUC_CAN_FRAME frame;
	int i, ret;
	struct sk_buff *skb;
	struct can_frame cf;

	memset(&frame, 0, sizeof(frame));

	memcpy(frame.com_buf, info->rbuff, info->rcount);
	frame.com_buf[info->rcount] = '\r';
	frame.com_buf[info->rcount + 1] = '\n';

	//printk("emuc : bump : %s\n", frame.com_buf);

	if ((ret = EMUCRevHex(&frame)) < 0) {
		printk("emuc : bump : parse fail %d.\n", ret);
		return;
	}

	if (frame.channel != EMUC_CH_1 && frame.channel != EMUC_CH_2) {
		printk("emuc : bump : invalid channel %d\n", frame.channel);
		return;
	}

	cf.can_id = 0;
	if (frame.rtr) cf.can_id |= CAN_RTR_FLAG;
	if (frame.mod) cf.can_id |= CAN_EFF_FLAG;

	for (i = 0; i < ID_LEN; i++)
		cf.can_id |= (frame.id[ID_LEN - 1 - i] << (i * 8));

	/* RTR frames may have a dlc > 0 but they never have any data bytes */
	*(u64 *)(&cf.data) = 0;
	if (!frame.rtr) {
		cf.can_dlc = frame.dlc;
		for (i = 0; i < DATA_LEN; i++)
			cf.data[i] = frame.data[i];
	} else
		cf.can_dlc = 0;

	//printk("emuc : bump : ID(%X) DLC(%d)\n", cf.can_id, cf.can_dlc);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	skb = dev_alloc_skb(sizeof(struct can_frame) +
			    sizeof(struct can_skb_priv));
#else
	skb = dev_alloc_skb(sizeof(struct can_frame));
#endif
	if (!skb)
		return;

	skb->dev = info->devs[frame.channel - 1];
	skb->protocol = htons(ETH_P_CAN);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	can_skb_reserve(skb);
	can_skb_prv(skb)->ifindex = info->devs[frame.channel - 1]->ifindex;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,5)
	can_skb_prv(skb)->skbcnt = 0;
#endif

	memcpy(skb_put(skb, sizeof(struct can_frame)), &cf, sizeof(struct can_frame));	

	info->devs[frame.channel - 1]->stats.rx_packets++;
	info->devs[frame.channel - 1]->stats.rx_bytes += cf.can_dlc;

	netif_rx_ni(skb);
}

/* parse tty input stream */
static void emuc_unesc(struct emuc *info, unsigned char s)
{
	if ((s == '\r') || (s == '\a') || (s == '\n')) { /* CR, RF or BEL ends the pdu */
		if (!test_and_clear_bit(SLF_ERROR, &info->flags) &&
		    (info->rcount > 4))  {
			emuc_bump(info);
		}
		info->rcount = 0;
	} else {
		if (!test_bit(SLF_ERROR, &info->flags))  {
			if (info->rcount < EMUC_MTU)  {
				info->rbuff[info->rcount++] = s;
				return;
			} else {
				info->devs[0]->stats.rx_over_errors++;
				info->devs[1]->stats.rx_over_errors++;
				set_bit(SLF_ERROR, &info->flags);
			}
		}
	}
}

 /************************************************************************
  *			STANDARD EMUC ENCAPSULATION			 *
  ************************************************************************/

/* Encapsulate one can_frame and stuff into a TTY queue. */
static void emuc_encaps(struct emuc *info, int channel, struct can_frame *cf)
{
	int i, len, actual;
	canid_t id = cf->can_id;
	EMUC_CAN_FRAME emuc_can_frame;

	memset(&emuc_can_frame, 0, sizeof(EMUC_CAN_FRAME));
	emuc_can_frame.channel = channel + 1;
	emuc_can_frame.rtr = (cf->can_id & CAN_RTR_FLAG) ? 1: 0;

	if (cf->can_id & CAN_EFF_FLAG) {
		emuc_can_frame.mod = 1;
		id &= cf->can_id & CAN_EFF_MASK;
	} else {
		emuc_can_frame.mod = 0;
		id &= cf->can_id & CAN_SFF_MASK;
	}
	for (i = ID_LEN - 1; i >= 0; i--) {
		emuc_can_frame.id[i] = id & 0xff;
		id >>= 8;
	}
	
	emuc_can_frame.dlc = cf->can_dlc;
	for (i = 0; i < cf->can_dlc; i++)
		emuc_can_frame.data[i] = cf->data[i];

	EMUCSendHex(&emuc_can_frame);
	len = strlen(emuc_can_frame.com_buf);
	memcpy(info->xbuff, emuc_can_frame.com_buf, len);

	/* Order of next two lines is *very* important.
	 * When we are sending a little amount of data,
	 * the transfer may be completed inside the ops->write()
	 * routine, because it's running with interrupts enabled.
	 * In this case we *never* got WRITE_WAKEUP event,
	 * if we did not request it before write operation.
	 *       14 Oct 1994  Dmitry Gorodchanin.
	 */
	set_bit(TTY_DO_WRITE_WAKEUP, &info->tty->flags);
	actual = info->tty->ops->write(info->tty, info->xbuff, len);
    
    #if defined(FIXED_QUEUE_BUF)
        info->xleft += actual;
    #else
    	info->xleft = len - actual;
    #endif

	info->xhead = info->xbuff + actual;
	info->devs[channel]->stats.tx_bytes += cf->can_dlc;

	/* v1.8: for fixing tx_packet bug */
	info->current_channel = channel;

}

/* Write out any remaining transmit buffer. Scheduled when tty is writable */
static void emuc_transmit(struct work_struct *work)
{
	struct emuc *info = container_of(work, struct emuc, tx_work);
	int actual;

	spin_lock_bh(&info->lock);
	/* First make sure we're connected. */
	if (!info->tty || info->magic != EMUC_MAGIC ||
			(!netif_running(info->devs[0]) && !netif_running(info->devs[1]))) {
		spin_unlock_bh(&info->lock);
		return;
	}

	if (info->xleft <= 0)  {

		#if 0
			/* Now serial buffer is almost free & we can start
			 * transmission of another packet */
			if (netif_running(info->devs[0]))
				info->devs[0]->stats.tx_packets++;
			if (netif_running(info->devs[1]))
				info->devs[1]->stats.tx_packets++;
		#endif

		/* v1.8 */
		info->devs[info->current_channel]->stats.tx_packets++;


		clear_bit(TTY_DO_WRITE_WAKEUP, &info->tty->flags);
		spin_unlock_bh(&info->lock);
		if (netif_running(info->devs[0]))
			netif_wake_queue(info->devs[0]);
		if (netif_running(info->devs[1]))
			netif_wake_queue(info->devs[1]);
		return;
	}

	actual = info->tty->ops->write(info->tty, info->xhead, info->xleft);
	info->xleft -= actual;
	info->xhead += actual;
	spin_unlock_bh(&info->lock);
}

/*
 * Called by the driver when there's room for more data.
 * Schedule the transmit.
 */
static void emuc_write_wakeup(struct tty_struct *tty)
{
	struct emuc *info = tty->disc_data;

	schedule_work(&info->tx_work);
}

/* Send a can_frame to a TTY queue. */
static netdev_tx_t emuc_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct emuc *info = ((struct emuc_priv *)netdev_priv(dev))->info;
	int channel;

	if (skb->len != sizeof(struct can_frame))
		goto out;

	spin_lock(&info->lock);
	if (!netif_running(dev))  {
		spin_unlock(&info->lock);
		printk(KERN_WARNING "%s: xmit: iface is down\n", dev->name);
		goto out;
	}
	if (info->tty == NULL) {
		spin_unlock(&info->lock);
		goto out;
	}
	channel = (dev->base_addr & 0xF00) >> 8;
	if (channel > 1) {
		spin_unlock(&info->lock);
		printk(KERN_WARNING "%s: xmit: invalid channel\n", dev->name);
		goto out;
	}

	netif_stop_queue(info->devs[0]);
	netif_stop_queue(info->devs[1]);
	emuc_encaps(info, channel, (struct can_frame *) skb->data); /* encaps & send */
	spin_unlock(&info->lock);

out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}


/******************************************
 *   Routines looking at netdevice side.
 ******************************************/

/* Netdevice UP -> DOWN routine */
static int emuc_netdev_close(struct net_device *dev)
{
	struct emuc *info = ((struct emuc_priv *)netdev_priv(dev))->info;
	int channel;


	channel = (dev->base_addr & 0xF00) >> 8;
	if (channel > 1) {
		printk(KERN_WARNING "%s: close: invalid channel\n", dev->name);
		return -1;
	}

	spin_lock_bh(&info->lock);
	if (info->tty) {
		/* TTY discipline is running. */
		if (!netif_running(info->devs[!channel]))
			clear_bit(TTY_DO_WRITE_WAKEUP, &info->tty->flags);
	}
	netif_stop_queue(dev);
	if (!netif_running(info->devs[!channel])) {
		/* another netdev is closed (down) too, reset TTY buffers. */
		info->rcount   = 0;
		info->xleft    = 0;
	}
	spin_unlock_bh(&info->lock);

	return 0;
}

/* Netdevice DOWN -> UP routine */
static int emuc_netdev_open(struct net_device *dev)
{
	struct emuc *info = ((struct emuc_priv *)netdev_priv(dev))->info;

	if (info->tty == NULL)
		return -ENODEV;

	info->flags &= (1 << SLF_INUSE);
	netif_start_queue(dev);
	return 0;
}

/* Hook the destructor so we can free emuc devs at the right point in time */
static void emuc_free_netdev(struct net_device *dev)
{
	struct emuc *info = ((struct emuc_priv *)netdev_priv(dev))->info;
	int i = (dev->base_addr & 0xFF);

	free_netdev(dev);
	emuc_devs[i] = NULL;
	if (atomic_dec_and_test(&info->ref_count)) {
		printk("free_netdev: free info\n");
		kfree(info);
	}
}

static int emuc_change_mtu(struct net_device *dev, int new_mtu)
{
	return -EINVAL;
}

static const struct net_device_ops emuc_netdev_ops = {
	.ndo_open               = emuc_netdev_open,
	.ndo_stop               = emuc_netdev_close,
	.ndo_start_xmit         = emuc_xmit,
	.ndo_change_mtu		= emuc_change_mtu,
};

static void emuc_setup(struct net_device *dev)
{
	dev->netdev_ops		= &emuc_netdev_ops;

	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,9)
	dev->priv_destructor = emuc_free_netdev;
	#else
	dev->destructor = emuc_free_netdev;
	#endif



	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 10;

	dev->mtu		= sizeof(struct can_frame);
	dev->type		= ARPHRD_CAN;

	/* New-style flags. */
	dev->flags		= IFF_NOARP;
	dev->features           = NETIF_F_HW_CSUM;
}

/******************************************
  Routines looking at TTY side.
 ******************************************/

/*
 * Handle the 'receiver data ready' interrupt.
 * This function is called by the 'tty_io' module in the kernel when
 * a block of EMUC data has been received, which can now be decapsulated
 * and sent on to some IP layer for further processing. This will not
 * be re-entered while running but other ldisc functions may be called
 * in parallel
 */

static void emuc_receive_buf(struct tty_struct *tty,
			      const unsigned char *cp, char *fp, int count)
{
	struct emuc *info = (struct emuc *) tty->disc_data;

	if (!info || info->magic != EMUC_MAGIC ||
			(!netif_running(info->devs[0]) && !netif_running(info->devs[1])))
		return;

	/* Read the characters out of the buffer */
	while (count--) {
		if (fp && *fp++) {
			if (!test_and_set_bit(SLF_ERROR, &info->flags)) {
				if (netif_running(info->devs[0]))
					info->devs[0]->stats.rx_errors++;
				if (netif_running(info->devs[1]))
					info->devs[1]->stats.rx_errors++;
			}
			cp++;
			continue;
		}
		emuc_unesc(info, *cp++);
	}
}

/************************************
 *  emuc_open helper routines.
 ************************************/

/* Collect hanged up channels */
static void emuc_sync(void)
{
	int i;
	struct net_device *dev;
	struct emuc	  *info;

	for (i = 0; i < maxdev; i++) {
		dev = emuc_devs[i];
		if (dev == NULL)
			break;

		info = ((struct emuc_priv *)netdev_priv(dev))->info;
		if (info->tty)
			continue;
		if (dev->flags & IFF_UP)
			dev_close(dev);
	}
}

/* Find a free EMUC channel, and link in this `tty' line. */
static int emuc_alloc(dev_t line, struct emuc *info)
{
	int i, channel, id[2];
	char name[IFNAMSIZ];
	struct net_device *dev;
	struct net_device *devs[2];
	struct emuc_priv  *priv;
	
	channel = 0;
	for (i = 0; i < maxdev; i++) {
		dev = emuc_devs[i];
		if (dev == NULL) {
			id[channel++] = i;
			if (channel > 1)
				break;
		}
	}

	/* Sorry, too many, all slots in use */
	if (i >= maxdev)
		return -1;

	sprintf(name, "emuccan%d", id[0]);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
	devs[0] = alloc_netdev(sizeof(*priv), name, emuc_setup);
#else
	devs[0] = alloc_netdev(sizeof(*priv), name, NET_NAME_UNKNOWN, emuc_setup);
#endif
	if (!devs[0])
		return -1;

	sprintf(name, "emuccan%d", id[1]);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
	devs[1] = alloc_netdev(sizeof(*priv), name, emuc_setup);
#else
	devs[1] = alloc_netdev(sizeof(*priv), name, NET_NAME_UNKNOWN, emuc_setup);
#endif
	if (!devs[1]) {
		free_netdev(devs[0]);
		return -1; 
	}

	devs[0]->base_addr = id[0];
	devs[1]->base_addr = 0x100 | id[1];

	priv = netdev_priv(devs[0]);
	priv->magic = EMUC_MAGIC;
	priv->info = info;
	priv = netdev_priv(devs[1]);
	priv->magic = EMUC_MAGIC;
	priv->info = info;

	/* Initialize channel control data */
	info->magic = EMUC_MAGIC;
	info->devs[0] = devs[0];
	info->devs[1] = devs[1];
	emuc_devs[id[0]] = devs[0];
	emuc_devs[id[1]] = devs[1];
	spin_lock_init(&info->lock);
	atomic_set(&info->ref_count, 2);
	INIT_WORK(&info->tx_work, emuc_transmit);

	return 0;
}

/*
 * Open the high-level part of the EMUC channel.
 * This function is called by the TTY module when the
 * EMUC line discipline is called for.  Because we are
 * sure the tty line exists, we only have to link it to
 * a free EMUC channel...
 *
 * Called in process context serialized from other ldisc calls.
 */

static int emuc_open(struct tty_struct *tty)
{
	struct emuc *info;
	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (tty->ops->write == NULL)
		return -EOPNOTSUPP;

	/* RTnetlink lock is misused here to serialize concurrent
	   opens of emuc channels. There are better ways, but it is
	   the simplest one.
	 */
	rtnl_lock();

	/* Collect hanged up channels. */
	emuc_sync();

	info = tty->disc_data;

	err = -EEXIST;
	/* First make sure we're not already connected. */
	if (info && info->magic == EMUC_MAGIC)
		goto err_exit;

	/* OK. Allocate emuc info. */
	err = -ENOMEM;
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		goto err_exit;

	/* OK.  Find a free EMUC channel to use. */
	err = -ENFILE;
	if (emuc_alloc(tty_devnum(tty), info) != 0) {
		kfree(info);
		goto err_exit;
	}

	info->tty = tty;
	tty->disc_data = info;

	if (!test_bit(SLF_INUSE, &info->flags)) {
		/* Perform the low-level EMUC initialization. */
		info->rcount   = 0;
		info->xleft    = 0;

		set_bit(SLF_INUSE, &info->flags);

		err = register_netdevice(info->devs[0]);
		if (err)
			goto err_free_chan;

		err = register_netdevice(info->devs[1]);
		if (err) {
			unregister_netdev(info->devs[0]);
			goto err_free_chan;
		}
	}

	/* Done.  We have linked the TTY line to a channel. */
	rtnl_unlock();
	tty->receive_room = 65536;	/* We don't flow control */

	/* TTY layer expects 0 on success */
	return 0;

err_free_chan:
	info->tty = NULL;
	tty->disc_data = NULL;
	clear_bit(SLF_INUSE, &info->flags);

err_exit:
	rtnl_unlock();

	/* Count references from TTY module */
	return err;
}

/*
 * Close down a EMUC channel.
 * This means flushing out any pending queues, and then returning. This
 * call is serialized against other ldisc functions.
 *
 * We also use this method for a hangup event.
 */

static void emuc_close(struct tty_struct *tty)
{
	struct emuc *info = (struct emuc *) tty->disc_data;

	/* First make sure we're connected. */
	if (!info || info->magic != EMUC_MAGIC || info->tty != tty)
		return;

	spin_lock_bh(&info->lock);
	tty->disc_data = NULL;
	info->tty = NULL;
	spin_unlock_bh(&info->lock);

	flush_work(&info->tx_work);

	/* Flush network side */
	unregister_netdev(info->devs[0]);
	unregister_netdev(info->devs[1]);
	/* This will complete via emuc_free_netdev */
}

static int emuc_hangup(struct tty_struct *tty)
{
	emuc_close(tty);
	return 0;
}

/* Perform I/O control on an active EMUC channel. */
static int emuc_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct emuc *info = (struct emuc *) tty->disc_data;
	unsigned int tmp;
	int channel;

	/* First make sure we're connected. */
	if (!info || info->magic != EMUC_MAGIC)
		return -EINVAL;

	switch (cmd) {
	case SIOCGIFNAME:
		channel = info->gif_channel;
		tmp = strlen(info->devs[channel]->name) + 1;
		if (copy_to_user((void __user *)arg, info->devs[channel]->name, tmp))
			return -EFAULT;
		info->gif_channel = !info->gif_channel;
		return 0;

	case SIOCSIFHWADDR:
		return -EINVAL;

	default:
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}

static struct tty_ldisc_ops emuc_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "emuccan",
	.open		= emuc_open,
	.close		= emuc_close,
	.hangup		= emuc_hangup,
	.ioctl		= emuc_ioctl,
	.receive_buf	= emuc_receive_buf,
	.write_wakeup	= emuc_write_wakeup,
};


static int __init emuc_init(void)
{
	int status;

	if (maxdev < 4)
		maxdev = 4; /* Sanity */

	printk(banner);
	printk(KERN_INFO "emuc: %d dynamic interface channels.\n", maxdev);

	emuc_devs = kzalloc(sizeof(struct net_device *)*maxdev, GFP_KERNEL);
	if (!emuc_devs)
		return -ENOMEM;

	/* Fill in our line protocol discipline, and register it */
	status = tty_register_ldisc(N_EMUC, &emuc_ldisc);
	if (status)  {
		printk(KERN_ERR "emuc: can't register line discipline\n");
		kfree(emuc_devs);
	}
	return status;
}

static void __exit emuc_exit(void)
{
	int i;
	struct net_device *dev;
	struct emuc *info;
	unsigned long timeout = jiffies + HZ;
	int busy = 0;

	if (emuc_devs == NULL)
		return;

	/* First of all: check for active disciplines and hangup them.
	 */
	do {
		if (busy)
			msleep_interruptible(100);

		busy = 0;
		for (i = 0; i < maxdev; i++) {
			dev = emuc_devs[i];
			if (!dev)
				continue;
			info = ((struct emuc_priv *)netdev_priv(dev))->info;
			spin_lock_bh(&info->lock);
			if (info->tty) {
				busy++;
				tty_hangup(info->tty);
			}
			spin_unlock_bh(&info->lock);
		}
	} while (busy && time_before(jiffies, timeout));

	/* FIXME: hangup is async so we should wait when doing this second
	   phase */

	for (i = 0; i < maxdev; i++) {
		dev = emuc_devs[i];
		if (!dev)
			continue;
		emuc_devs[i] = NULL;

		info = ((struct emuc_priv *)netdev_priv(dev))->info;
		if (info->tty) {
			printk(KERN_ERR "%s: tty discipline still running\n",
			       dev->name);

			/* Intentionally leak the control block. */
			#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,9)
			dev->priv_destructor = NULL;
			#else
			dev->destructor = NULL;
			#endif
		}

		unregister_netdev(dev);
	}

	kfree(emuc_devs);
	emuc_devs = NULL;

	i = tty_unregister_ldisc(N_EMUC);
	if (i)
		printk(KERN_ERR "emuc: can't unregister ldisc (err %d)\n", i);
}

module_init(emuc_init);
module_exit(emuc_exit);

/* vim: set noexpandtab ts=8 sw=8: */
