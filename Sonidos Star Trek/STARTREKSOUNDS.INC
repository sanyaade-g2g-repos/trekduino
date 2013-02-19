; external declaration of sound table:

    .EXTRN (const) SNDTBL_STARTREKSOUNDS



; external declarations of addresses of sounds for direct access:

    .EXTRN (const) SX_alarm1_shipWideAlert
    .EXTRN (const) SX_inputok2_inputSuccessful
    .EXTRN (const) SX_phasertype2_phaserTypeIII
    .EXTRN (const) SX_photorp3
    .EXTRN (const) SX_shieldraise
    .EXTRN (const) SX_voiceinput1_readyForInput1


; equates defining sounds as entries in the sound table:

SND_alarm1_shipWideAlert .EQU 1
SND_inputok2_inputSuccessful .EQU 2
SND_phasertype2_phaserTypeIII .EQU 3
SND_photorp3            .EQU 4
SND_shieldraise         .EQU 5
SND_voiceinput1_readyForInput1 .EQU 6

