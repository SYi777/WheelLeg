        AREA RESET, DATA, READONLY
        EXPORT __Vectors
__Vectors
        DCD 0x20010000
        DCD Reset_Handler
        DCD Default_Handler
        DCD Default_Handler
        DCD Default_Handler
        DCD Default_Handler
        DCD Default_Handler
        DCD 0,0,0,0
        DCD Default_Handler
        DCD Default_Handler
        DCD 0
        DCD Default_Handler
        DCD Default_Handler
        AREA |.text|, CODE, READONLY
        THUMB
        EXPORT Reset_Handler
        IMPORT __main
Reset_Handler
        LDR R0, =0xE000ED88
        LDR R1, [R0]
        ORR R1, R1, #0x00F00000
        STR R1, [R0]
        DSB
        ISB
        B __main
Default_Handler
        B Default_Handler
        END
