# Whisper

This repository contains the source code of Whisper. Whisper floods small amounts of data reliably into a multi-hop network. [[paper](https://dl.acm.org/citation.cfm?id=3356341)]

## Code Layout

*Disclaimer: Although we tested the code extensively, Whisper is a research prototype that likely contains bugs. We take no responsibility for and give no warranties in respect of using the code.*

We started the implementation using LaneFlood (see the code [here](laneflood)). We use LaneFlood's bootstrapping process but once it is finished, we switch to another scheduler and do not use LaneFlood anymore. For bootstrapping, we use Glossy. Because Glossy and Whisper use the same interrupt routine, we set the protocol to use with `set_gfast_modus(..)` to either `LANEFLOOD_GLOSSY` or `LANEFLOOD_WHISPER` (see `net/mac/gfast.h`). More precisely, the interrupt is caught in `net/mac/gfast.c` and the corresponding protocol is called from there. The implementation for LaneFlood can be found in `net/mac/laneflood`, the implementation for Glossy can be found in `net/mac/glossy`, and the implementation for Whisper can be found in `net/mac/whisper`.

Whisper does not work with IP yet. Therefore IP, TCP and UDP must be disabled: `CFLAGS += -DUIP_CONF_UDP=0 -DUIP_CONF_TCP=0 -DNETSTACK_CONF_WITH_IPV6=0`.

## Notes

The evaluation of this work was done with `msp430-gcc (GCC) 4.6.3 20120301 (mspgcc LTS 20120406 unpatched)`. Other compilers or versions of the msp430-gcc may lead to different evaluation results.
