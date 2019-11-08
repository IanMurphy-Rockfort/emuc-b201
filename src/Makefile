
MACHINE := $(shell uname -m)
EMUCINC := $(EMUCDIR)/include
EMUCLIB := lib/$(MACHINE)

obj-m += emuccan.o
#emuccan-y := emuc.o ../$(EMUCLIB)/libemuccan.a
emuccan-y :=emuc.o lib_emuccan.o
CFLAGS_emuc.o = -I$(EMUCINC)
CFLAGS_lib_emuccan.o = -I$(EMUCINC)

hostprogs-y := emucd
always      := $(hostprogs-y)
HOSTCFLAGS_emucd.o := -I$(EMUCINC)
HOSTLOADLIBES_emucd = -L$(EMUCDIR)/$(EMUCLIB) -lemuc -lpthread
