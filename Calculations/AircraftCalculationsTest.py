import aerosandbox as asb
import aerosandbox.numpy as np


#create main wing
wing_airfoil = asb.Airfoil("sd7037")
mW_X_Root = asb.WingXSec(xyz_le=[0,0,0],chord=0.5,twist=2,airfoil=wing_airfoil)
mW_X_1 = asb.WingXSec(xyz_le=[0,1,0],chord=0.3, twist=0, airfoil=wing_airfoil)
mW_X_2 = asb.WingXSec(xyz_le=[0,2,0],chord=0.1,twist=0, airfoil=wing_airfoil)
main_wing = asb.Wing(name="Main Wing",symmetric=True,xsecs=[mW_X_Root,mW_X_1,mW_X_2])

#Fuselage
Fuse_X_arr = []
for i in range(0,11):
    _radius = asb.Airfoil("dae51").local_thickness(0.1 * i)
    Fuse_X_arr.append(asb.FuselageXSec(xyz_c=[i*0.12,0,0],radius=_radius))
fuselage = asb.Fuselage(name="Fuselage",xsecs=Fuse_X_arr).translate([-0.5,0,0])

plane = asb.Airplane(name="TestPlane",xyz_ref=[0,0,0],wings=[main_wing],fuselages=[fuselage])

plane.draw_three_view()

aeroTest = asb.AeroBuildup(
    airplane=plane,
    op_point=asb.OperatingPoint(
        velocity=10,
        alpha=5,
    ),
).run()
