n = 0
while n < 1:
    print("n: ",n)
    n += 1

    # Try leg to a desired position
    x4 = 0
    y4 = -0.18
    z4 = 00.05

    for i in range(0, 40, 8):
        theta = 10*d2r  #pitch
        psi   = 0*d2r   #yaw
        phi   = 0*d2r   #roll

        arr= np.array([ [rrx+i/1000,   rry+0.03,  rrz],
                        [rfx ,  rfy,  rfz],
                        [lfx ,  lfy,  lfz],
                        [lrx ,  lry,  lrz] ])

        (q1,q2,q3) = smk.ikine(rrx+i/1000,rry+0.03,rrz,l1,l2,l3,legs12 = True)

        print('Leg angles')
        print('q1: %2.1f deg, q2: %2.1f deg, q3: %2.1f deg'%(q1*r2d,q2*r2d,q3*r2d))

        plotme(theta, psi, phi, arr)
        #time.sleep(0.2)
        print(i)


    for i in range(40, -40, -8):
        theta = 10*d2r  #pitch
        psi   = 0*d2r   #yaw
        phi   = 0*d2r   #roll

        arr= np.array([ [rrx+i/1000,   rry,  rrz],
                        [rfx ,  rfy,  rfz],
                        [lfx ,  lfy,  lfz],
                        [lrx ,  lry,  lrz] ])

        plotme(theta, psi, phi, arr)
        #time.sleep(0.2)
        print(i)

    for i in range(-40, 0, 8):
        theta = 10*d2r  #pitch
        psi   = 0*d2r   #yaw
        phi   = 0*d2r   #roll

        arr= np.array([ [rrx+i/1000,   rry+0.03,  rrz],
                        [rfx ,  rfy,  rfz],
                        [lfx ,  lfy,  lfz],
                        [lrx ,  lry,  lrz] ])

        plotme(theta, psi, phi, arr)
        #time.sleep(0.2)
        print(i)
