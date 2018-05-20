from machine import I2C
import utime, math

Freq = 1000
Delay = (1/Freq)*1000


def average(P1,P2):
    i2c = I2C(0,pins=(P1,P2))
    i2c.writeto_mem(0x53, 0x2C, 0x0A)
    i2c.writeto_mem(0x53, 0x2D, 0x08)
    i2c.writeto_mem(0x53, 0x31, 0x08)
    i2c.writeto_mem(0x53, 0x2C, 0x0F)
    scale=0.004

    AcX = 0
    AmX = 0
    BmX = 0
    MmX = 0
    LmX = 0

    x = 0
    xv = [0,0,0]
    yv = [0,0,0]
    Bxv = [0,0,0]
    Byv = [0,0,0]
    Lxv = [0,0,0]
    Lyv = [0,0,0]
    Mxv = [0,0,0]
    Myv = [0,0,0]

    for i in range(Freq):

        ## SET FREQUENCY IN MICROSECONDS
        now = utime.ticks_us()
        ## TAKE SAMPLE AND RANGE TO G-s
        byte=i2c.readfrom_mem(0x53, 0x32, 2)

        x=byte[0] | (byte[1] << 8)
        if(x & (1 << 16 - 1)):
            x = x - (1<<16)

        #y = byte[2] | (byte[3] << 8)
        #if(y & (1 << 16 - 1)):
            #y = y - (1<<16)

        #z = byte[4] | (byte[5] << 8 )
        #if(z & (1 << 16 - 1)):
    	    #z = z - (1<<16)

        x = x*scale
        #y = y*scale
        #z = z*scale

        OallGAIN = 1.006278364e+00
        xv[0] = xv[1]
        xv[1] = xv[2]
        xv[2] = x / OallGAIN
        yv[0] = yv[1]
        yv[1] = yv[2]
        yv[2] = (xv[2] - xv[0]) + (0.9875119299 * yv[0]) + ( -0.0062440659 * yv[1])

        BrgsGAIN = 1.995709830e+00
        Bxv[0] = Bxv[1]
        Bxv[1] = Bxv[2]
        Bxv[2] = x / BrgsGAIN
        Byv[0] = Byv[1]
        Byv[1] = Byv[2]
        Byv[2] = (Bxv[2] - Bxv[0]) + (-0.0000000000 * Byv[0]) + ( -0.2212317421 * Byv[1])

        MechGAIN = 6.804848824e+00
        Mxv[0] = Mxv[1]
        Mxv[1] = Mxv[2]
        Mxv[2] = x / MechGAIN
        Myv[0] = Myv[1]
        Myv[1] = Myv[2]
        Myv[2] = (Mxv[2] - Mxv[0]) + ( -0.7337672067 * Myv[0]) + ( 1.7311810572 * Myv[1])

        LowfGAIN = 2.711530715e+00
        Lxv[0] = Lxv[1]
        Lxv[1] = Lxv[2]
        Lxv[2] = x / LowfGAIN
        Lyv[0] = Lyv[1]
        Lyv[1] = Lyv[2]
        Lyv[2] = (Lxv[2] - Lxv[0]) + ( -0.3301376134 * Lyv[0]) + ( 1.3237653273 * Lyv[1])

        AcX = AcX + math.pow(yv[2]*9.81*1000,2)
        BmX = BmX + math.pow(Byv[2]*9.81*1000,2)
        MmX = MmX + math.pow(Myv[2]*9.81*1000,2)
        LmX = LmX + math.pow(Lyv[2]*9.81*1000,2)

        ## PAUSE FOR FREQUENCY
        while utime.ticks_us() < now+Delay:
            pass

    oall = round((math.sqrt(AcX/Freq)/(2*3.142*Freq)),3)
    brgs = round((math.sqrt(BmX/Freq)/(2*3.142*600)),3)
    mech = round((math.sqrt(MmX/Freq)/(2*3.142*100)),3)
    lowf = round((math.sqrt(LmX/Freq)/(2*3.142*300)),3)

    #LORA / WIFI / LTE PACKET HASH IF USING SIGFOX
    return(oall,brgs,mech,lowf)

def data(P1,P2):

    sample = []
    aoall = 0
    abrgs = 0
    amech = 0
    alowf = 0

    for i in range(4):
        sample = average(P1,P2)
        aoall = sample[0]+aoall
        abrgs = sample[1]+abrgs
        amech = sample[2]+amech
        alowf = sample[3]+alowf
    
    return('{"val":'+str(aoall/4)+',"brgs":'+str(abrgs/4)+',"mech":'+str(amech/4)+',"lowf":'+str(alowf/4)+'}')

## LOOP FOR TESTING - UNHASH BELOW TO TEST ##
#while True:
    #print(data('P22','P21'))
