# invoke SourceDir generated makefile for SimpleUART.prm3
SimpleUART.prm3: .libraries,SimpleUART.prm3
.libraries,SimpleUART.prm3: package/cfg/SimpleUART_prm3.xdl
	$(MAKE) -f C:\Users\johnny\Desktop\UartOverAir/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\johnny\Desktop\UartOverAir/src/makefile.libs clean

