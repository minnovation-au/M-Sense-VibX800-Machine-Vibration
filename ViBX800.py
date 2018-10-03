##||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||##
##.....................VIBX800.  Vibration Monitor .........................##
##....................Machine Vibration Monitoring @ 3 - 1000 Hz............##
##......................Written by Simon Maselli............................##
##......................www.minnovation.com.au..............................##
##......................January 27,2018.....................................##
##..................... Copyright 2018 - M innovation.......................##
##||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||##

from machine import I2C, Timer
import utime, math

Freq = 400
Delay = (1000/(Freq*2))*1000

chrono = Timer.Chrono()

def data(P1,P2):
    i2c = I2C(0,pins=(P1,P2))
    i2c.writeto_mem(0x53, 0x2C, 0x0D)
    i2c.writeto_mem(0x53, 0x2D, 0x08)
    i2c.writeto_mem(0x53, 0x31, 0x03)
    i2c.writeto_mem(0x53, 0x38, 0x0)

    scale=0.032

    AcX = 0
    AmX = 0
    BmX = 0
    MmX = 0
    LmX = 0

    time=0
    g=0
    xo=0
    repeat=0

    x = 0
    xavg=0

    xv = [0,0,0]
    yv = [0,0,0]
    Bxv = [0,0,0]
    Byv = [0,0,0]
    Lxv = [0,0,0]
    Lyv = [0,0,0]
    Mxv = [0,0,0]
    Myv = [0,0,0]

    chrono.start()
    for i in range(Freq*2):

        ## SET FREQUENCY IN MICROSECONDS

        ## TAKE SAMPLE AND RANGE TO G-s
        byte=i2c.readfrom_mem(0x53, 0x32, 2)

        x=byte[0] | (byte[1] << 8)
        if(x & (1 << 16 - 1)):
            x = x - (1<<16)

        x=(x/30)
        xavg = xavg+x
        #print(x)
        #y = byte[2] | (byte[3] << 8)
        #if(y & (1 << 16 - 1)):
            #y = y - (1<<16)

        #print(y/31.2)
        #z = byte[4] | (byte[5] << 8 )
        #if(z & (1 << 16 - 1)):
            #z = z - (1<<16)

        #y = y*scale
        #z = z*scale
        #print(x)

        ### 5-400HZ FILTER RANGE ####
        OallGAIN = 1.005881289e+00
        xv[0] = xv[1]
        xv[1] = xv[2]
        xv[2] = x / OallGAIN
        yv[0] = yv[1]
        yv[1] = yv[2]
        yv[2] = (xv[2] - xv[0]) + (0.9883024188 * yv[0]) + ( -0.0038992137 * yv[1])

        ### 200 - 400HZ FILTER RANGE ####
        BrgsGAIN = 1.851530787e+00
        Bxv[0] = Bxv[1]
        Bxv[1] = Bxv[2]
        Bxv[2] = x / BrgsGAIN
        Byv[0] = Byv[1]
        Byv[1] = Byv[2]
        Byv[2] = (Bxv[2] - Bxv[0]) + ( -0.0009805224 * Byv[0]) + ( -0.9970545910 * Byv[1])

        ### 1-100HZ Filter Range ###
        LowfGAIN = 3.117370539e+00
        Lxv[0] = Lxv[1]
        Lxv[1] = Lxv[2]
        Lxv[2] = x / LowfGAIN
        Lyv[0] = Lyv[1]
        Lyv[1] = Lyv[2]
        Lyv[2] = (Lxv[2] - Lxv[0]) + ( -0.4193924499 * Lyv[0]) + ( 1.4147944404 * Lyv[1])

        ### 5 - 200HZ FILTER RANGE ####
        MechGAIN = 1.904102778e+00
        Mxv[0] = Mxv[1]
        Mxv[1] = Mxv[2]
        Mxv[2] = x / MechGAIN
        Myv[0] = Myv[1]
        Myv[1] = Myv[2]
        Myv[2] = (Mxv[2] - Mxv[0]) + ( -0.0205938737 * Myv[0]) + ( 0.9814056714 * Myv[1])

        xo = xo + math.pow(x,2)
        AcX = AcX + math.pow(yv[2],2)
        BmX = BmX + math.pow(Byv[2],2)
        MmX = MmX + math.pow(Myv[2],2)
        LmX = LmX + math.pow(Lyv[2],2)

        ## PAUSE FOR FREQUENCY

        g=g+1
        while chrono.read_us() < Delay:
            repeat=repeat+1
            pass
        time=time+chrono.read_ms()
        chrono.reset()

    i2c.writeto_mem(0x53, 0x2D, 0x0)

    print("Delay Seconds: ",Delay)
    print("Total Cycles: ",repeat)
    print("Total Samples",g)
    print("Total Time (ms):",time)
    xavg = xavg/(Freq*2)
    xavg = math.sqrt( math.pow(xavg,2))
    print("Average all Samples: ",xavg)

    MSG = xo/(Freq*2)
    print("Mean Squared G: ",MSG)
    MSG = MSG - xavg
    MSG = math.sqrt(math.pow(MSG,2))
    print(MSG)
    print("Mean Squared G-AVG: ",(xo/(Freq*2))-(xavg))

    print("Mean Squared m/s: ",xo/(Freq*2)*9.81)
    print("Mean Squared mm/s: ",xo/(Freq*2)*9.81*1000)
    print("Mean Squared mm/s V: ",(xo/(Freq*2)*9.81*1000)/(2*3.142*Freq))
    RMSG=math.sqrt(xo/(Freq*2))
    print("Root Mean Squared G: ",RMSG)
    RMSV=(math.sqrt(xo/(Freq*2))*9.81*1000)/(2*3.142*Freq)
    print("Root Mean Squared mm/s V: ",RMSV)
    xo=((xo*9.81/(Freq*2))*1000)/(3.142*2*Freq)

    oall = round(math.sqrt(AcX/(Freq*2))*9.81*1000/(3.142*2*Freq),3)
    brgs = round(math.sqrt(BmX/(Freq*2))*9.81*1000/(3.142*2*Freq),3)
    mech = round(math.sqrt(MmX/(Freq*2))*9.81*1000/(3.142*2*Freq),3)
    lowf = round(math.sqrt(LmX/(Freq*2))*9.81*1000/(3.142*2*Freq),3)

    #LORA / WIFI / LTE PACKET HASH IF USING SIGFOX
    return('{"val":'+str(oall)+',"oall":'+str(MSG)+',"brgs":'+str(brgs)+',"mech":'+str(mech)+',"lowf":'+str(lowf)+'}')



## LOOP FOR TESTING - UNHASH BELOW TO TEST ##
#while True:
    #print(data('P10','P19'))
