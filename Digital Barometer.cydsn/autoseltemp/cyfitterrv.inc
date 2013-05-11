    IF :LNOT::DEF:INCLUDED_CYFITTERRV_INC
INCLUDED_CYFITTERRV_INC EQU 1
    GET cydevicerv.inc
    GET cydevicerv_trm.inc

; Miscellaneous
; -- WARNING: define names containing LEOPARD or PANTHER are deprecated and will be removed in a future release
CYDEV_BOOTLOADER_ENABLE EQU 0
    ENDIF
    END
