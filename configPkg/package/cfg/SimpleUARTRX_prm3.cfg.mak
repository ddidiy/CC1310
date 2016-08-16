# invoke SourceDir generated makefile for SimpleUARTRX.prm3
SimpleUARTRX.prm3: .libraries,SimpleUARTRX.prm3
.libraries,SimpleUARTRX.prm3: package/cfg/SimpleUARTRX_prm3.xdl
	$(MAKE) -f C:\Users\Yuefan\Documents\UartOverAir/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Yuefan\Documents\UartOverAir/src/makefile.libs clean

