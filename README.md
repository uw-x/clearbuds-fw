# shio

To build:

Copy nRF SDK into this repository
Update GNU_INSTALL_ROOT inside Makefile.posix
Note: There is an updated file nrfx_pdm.c which needs to be overwritten nRF SDK 

To test time sync on two nRF52840 DKs:

Press button 1 on board 1. This board will become the timing master.
Press button 2 on both master and slave at the same time. This arms the PPI to start the PDM upon the common sync clock reaching 0.
The PDM will then start.

## GDB Guide
To debug via GDB, run "make gdb" inside the top-level shio directory.  
