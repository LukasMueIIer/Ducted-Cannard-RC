import aerosandbox as asb
import aerosandbox.numpy as np

class fixedSweptWing:
    def __init__(self, opt, name = "Fixed Swept WIng", position = [0, 0, 0], sections = 10, iniSpan = 2, iniCord = 0.5, sweep = 0, twistLimit = 0, dihedral = 0) -> None:

        self.position = position
        self.name = name
        self.n = sections
        
        log = True

        ##OPT VARIABLES
        #fixed Values
        self.sweep = sweep
        self.dihedral = dihedral

        #scalar Values
        self.span = opt.variable(init_guess = iniSpan, log_transform = log, scale = 1)  #overall Spann, each section is the same distance

        #vectors
        self.chord = opt.variable(init_guess = np.ones(sections) * iniCord, log_transform = log, scale = 0.1)
        self.twist = opt.variable(init_guess = np.zeros(sections), scale = 0.1)

        
        #'OPT Conditions
        #scalars
        opt.subject_to(self.span > 0)

        #vectors

        for i in range(0,sections):
            if(i > 0):
                opt.subject_to(self.chord[i] <= self.chord[i-1])
            opt.subject_to(np.abs(self.twist[i]) < twistLimit)

        opt.subject_to(self.chord[-1] > 0.05)

        #Airfoil Shapes
        self.XSec = []  
        for i in range(0,sections):
            self.XSec.append(asb.Airfoil("sd7037"))

    def getWing(self,span,chord,sweep,dihedral,twist):  #Returns an asb wing object to attach to an aircraft

        n = self.n
        Xsec = []

        delta = span/(self.n - 1)

        for i in range(0,n):
            if(i == 0):
                x = self.position[0]
                y = 0
                z = self.position[2]
            else:
                x = x + 0.25 * chord[i - 1] + delta * np.sin(sweep / 180 * np.pi) - 0.25 * chord[i]
                y = y + delta * np.cos(sweep / 180 * np.pi )
                z = z + delta * np.sin(dihedral / 180 * np.pi )
            _c = chord[i]
            _t = twist[i]
            _foil = self.XSec[i]
            Xsec.append(asb.WingXSec(xyz_le=[x,y,z],chord=_c,twist=_t,airfoil=_foil))

        #create main wing
        wing = asb.Wing(name=self.name,symmetric=True,xsecs=Xsec)
        return wing
    
    def getWingOpt(self):   #wing for optimizer
        return self.getWing(self.span ,self.chord , self.sweep, self.dihedral, self.twist)
    
    def getWingEval(self,res):  #acutall wing
        span = res.value(self.span)
        chord = res.value(self.chord)
        sweep = self.sweep
        dihedral = self.dihedral
        twist = res.value(self.twist)
        return self.getWing(span,chord,sweep,dihedral,twist)
        

opt = asb.Opti()

#parameters
m0 = 1
v = 20

alpha = 8

main_wing = fixedSweptWing(opt, name = "Fixed Swept WIng", position = [0, 0, 0], sections = 5, iniSpan = 2, iniCord = 0.5, sweep = 0, twistLimit = 3, dihedral = 0)


#wings = [main_wing.getWing()]
plane = asb.Airplane(name="TestPlane",xyz_ref=[0,0,0],wings=[main_wing.getWingOpt()],fuselages=[])

m = m0 + main_wing.span * 5 + plane.s_ref * 5
lift = m * 9.81 

vlm = asb.VortexLatticeMethod(
    airplane=plane,
    op_point=asb.OperatingPoint(
        velocity=v,  # m/s
        alpha=alpha,  # degree
    ),spanwise_resolution=1,
    chordwise_resolution=6
)

aero = vlm.run()

opt.subject_to(aero["L"] == lift)
opt.minimize(aero["D"] + plane.s_ref)

rIter = []

#def cb():
 #   rIter.append(opti.debug)

res = opt.solve(verbose=True)

plane = asb.Airplane(name="ResultPlane",xyz_ref=[0,0,0],wings=[main_wing.getWingEval(res)],fuselages=[])
plane.draw_three_view()


vlm = asb.VortexLatticeMethod(
    airplane=plane,
    op_point=asb.OperatingPoint(
        velocity=v,  # m/s
        alpha=alpha,  # degree
    ),spanwise_resolution=1,
    chordwise_resolution=8
)
aero = vlm.run()
vlm.draw(show_kwargs=dict(jupyter_backend="static"))

print(res)