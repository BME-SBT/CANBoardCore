# MCP251x CAN timing utility

baudrate = 1E6
oscillator = 8E6


def get_sjw_and_brp(cnf1):
    sjw = (cnf1 & 0b11000000) >> 6
    brp = (cnf1 & 0b00111111)
    return (sjw, brp)


def cnf2_parse(cnf2, tq):
    btlmode = (cnf2 & 0b10000000) >> 7
    sam = (cnf2 & 0b01000000) >> 6
    phseg = (cnf2 & 0b00111000) >> 3
    prseg = (cnf2 & 0b00000111)

    return (btlmode, sam, phseg + 1, prseg + 1)


def cnf3_parse(cnf3, tq):
    wakeup = (cnf3 & 0b01000000) >> 6
    phseg2 = (cnf3 & 0b00000111)

    return (wakeup, (phseg2 + 1))


def calc_timing_brp(brp, oscillator):
    tq = 2 * (brp + 1) / oscillator
    return tq


def baud_from_timing(oscillator, brp):
    tq = 2 * (brp + 1) * oscillator


cnf1 = 0x00
cnf2 = 0x80
cnf3 = 0x86
sjw, brp = get_sjw_and_brp(cnf1)    # for 8Mhz, 5000 baud
tq = calc_timing_brp(brp, 8E6)
print("tq=", tq, ",sync width len=", sjw)
btlmode, sam, phseg, prseg = cnf2_parse(cnf2, tq)
print("phase2 in CNF3=", btlmode == 1, ", bus samples three times=",
      sam == 1, ", phseg=", phseg, ", prseg=", prseg)
wake, phseg2 = cnf3_parse(cnf3, tq)
print("wakfil=", wake == 1, ", phseg2=", phseg2)
