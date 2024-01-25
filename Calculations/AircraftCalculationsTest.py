import aerosandbox as asb
import aerosandbox.numpy as np

def createPlane(c0,c1,c2,x1,x2):
    #create main wing
    wing_airfoil = asb.Airfoil("sd7037")
    mW_X_Root = asb.WingXSec(xyz_le=[0,0,0],chord=c0,twist=2,airfoil=wing_airfoil)
    mW_X_1 = asb.WingXSec(xyz_le=x1,chord=c1, twist=0, airfoil=wing_airfoil)
    mW_X_2 = asb.WingXSec(xyz_le=x2,chord=c2,twist=0, airfoil=wing_airfoil)
    main_wing = asb.Wing(name="Main Wing",symmetric=True,xsecs=[mW_X_Root,mW_X_1,mW_X_2])

    #Fuselage
    Fuse_X_arr = []
    for i in range(0,11):
        _radius = asb.Airfoil("dae51").local_thickness(0.1 * i)
        Fuse_X_arr.append(asb.FuselageXSec(xyz_c=[i*0.12,0,0],radius=_radius))
    fuselage = asb.Fuselage(name="Fuselage",xsecs=Fuse_X_arr).translate([-0.5,0,0])

    return  asb.Airplane(name="TestPlane",xyz_ref=[0,0,0],wings=[main_wing],fuselages=[fuselage])

opt = asb.Opti()

targetLift = 90

#optimize wing geometry at CL of
alphaOPT = 5

#wing geometry
c0 = opt.variable(init_guess=0.5)
c1 = opt.variable(init_guess=0.5)
c2 = opt.variable(init_guess=0.5)

x1 = opt.variable(init_guess=[0,1,0])
x2 = opt.variable(init_guess=[0,2,0])

#plane = createPlane(0.5,0.5,0.5,[0,1,0],[0,2,0])

#aero = asb.AeroBuildup(
 #   airplane=plane,
  #  op_point=asb.OperatingPoint(
   #     velocity=10,
    #    alpha=4,
    #),
#).run()



#enforce geometric restrictions
opt.subject_to(x1[1] >= 0.05)
opt.subject_to(x2[1] > x1)

opt.subject_to(x1[2] == 0)
opt.subject_to(x2[2] == 0)


planeOPT = createPlane(c0,c1,c2,x1,x2)

aero = asb.AeroBuildup(
    airplane=planeOPT,
    op_point=asb.OperatingPoint(
        velocity=10,
        alpha=alphaOPT,
    ),
).run()

#opt.subject_to(aero["L"] == targetLift)
opt.minimize(aero["CD"])

res = opt.solve()

plane = createPlane(res.value(c0),res.value(c1),res.value(c2),res.value(x1),res.value(x2))

plane.draw_three_view()
