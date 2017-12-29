include Makefile.incl

CUDA_OBJS=

CPU_OBJS=BIDMachBullet.$(OBJ) 

.SUFFIXES: .$(OBJ) .c .cpp .cu

all: $(LIBPREPEND)bidmachbullet$(LIBAPPEND)

cudalib: 

cpulib: $(LIBPREPEND)bidmachbullet(LIBAPPEND) 

$(LIBPREPEND)bidmachbullet$(LIBAPPEND): $(CPU_OBJS)
	$(LD) $(LDFLAGS) $(CPU_OBJS) $(CPU_LIBS) $(BULLET_LIBS) $(OPENGL_LIBS) $(OUTFLG)$@

%.$(OBJ) : %.c
	$(CC) $(CFLAGS) $(LAPACK_INCLUDES) $*.c

%.$(OBJ) : %.cpp
	$(GCC) $(CPPFLAGS) $(LAPACK_INCLUDES) $*.cpp

install: installcpulib

installcpulib: $(INSTALLDIR)/$(LIBPREPEND)bidmachbullet$(LIBAPPEND)

$(INSTALLDIR)/$(LIBPREPEND)bidmachbullet$(LIBAPPEND): $(LIBPREPEND)bidmachbullet$(LIBAPPEND)
	mkdir -p $(INSTALL_DIR)
	cp $(LIBPREPEND)bidmachbullet$(LIBAPPEND) $(INSTALL_DIR)

jars: cpujar

cpujar: $(BIDMACH_ROOT)/target/BIDMachBullet-$(VERSION)-cpu-$(OS)-$(MARCH).jar

$(BIDMACH_ROOT)/target/BIDMachBullet-$(VERSION)-cpu-$(OS)-$(MARCH).jar: $(LIBPREPEND)bidmachbullet$(LIBAPPEND)
	cp $(LIBPREPEND)bidmachbullet$(LIBAPPEND) $(LIBDIR)
	jar cf $(BIDMACH_ROOT)/target/BIDMachBullet-$(VERSION)-cpu-$(OS)-$(MARCH).jar \
	-C $(BIDMACH_ROOT) lib/$(LIBPREPEND)bidmachbullet$(LIBAPPEND)
	rm $(LIBDIR)/$(LIBPREPEND)bidmachbullet$(LIBAPPEND)

clean:
	rm -f *.$(OBJ) *.pdb *.exp *.lib  *.jnilib *.so *.dylib *.dll 

distclean: clean
	rm -f Makefile.incl

cleanres:
	rm $(BIDMACH_ROOT)/src/main/resources/lib/*
