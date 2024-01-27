import aerosandbox as asb
import aerosandbox.numpy as np

class OptimizableWing:  #a subclass that makes optimization and mass management easier
    def __init__(self, opt, name="Wing",position = [0, 0, 0], sections = 10, iniSpan = 2, iniCord = 0.5, sweep = 0, twistLimit = 0, dihedralLimit = 0, forceEvenDistribution = True):

        self.position = position
        self.name = name
        self.n = sections
        
        log = True

        #changing section to span, chords, sweep and dihedral(as angle)
        self.span = opt.variable(init_guess = iniSpan, log_transform = log)  #overall Spann, each section is the same distance
        self.chord = opt.variable(init_guess = np.ones(sections) * iniCord, log_transform = log)
        self.sweep = np.ones(sections - 1) * sweep
        self.dihedral = np.zeros(sections - 1)
        self.twist = opt.variable(init_guess = np.zeros(sections))

        #create propper conditions
        opt.subject_to(self.span > 0)

        for i in range(0,sections):
            if(i > 0):
                opt.subject_to(self.chord[i] <= self.chord[i-1])
            if(i < sections - 1):
                opt.subject_to(np.abs(self.dihedral[i]) < dihedralLimit)
            opt.subject_to(np.abs(self.twist[i]) < twistLimit)
        opt.subject_to(self.chord[-1] > 0.05)

        #Airfoil Shapes
        self.XSec = []  
        for i in range(0,sections):
            self.XSec.append(asb.Airfoil("sd7037"))

    def getWing(self,span,chord,sweep,dihedral,twist):  #Returns an asb wing object to attach to an aircraft

        n = self.n
        Xsec = []

        for i in range(0,n):
            if(i == 0):
                x = self.position[0]
                y = 0
                z = self.position[2]
            else:
                x = x + 0.25 * chord[i - 1] + span[i-1] * np.sin(sweep[i-1] / 180 * np.pi) - 0.25 * chord[i]
                y = y + span[i-1] * np.cos(sweep[i-1] / 180 * np.pi )
                z = z + span[i-1] * np.sin(dihedral[i-1] / 180 * np.pi )
            _c = chord[i]
            _t = twist[i]
            _foil = self.XSec[i]
            Xsec.append(asb.WingXSec(xyz_le=[x,y,z],chord=_c,twist=_t,airfoil=_foil))

        #create main wing
        wing = asb.Wing(name=self.name,symmetric=True,xsecs=Xsec)
        return wing

    def getWingOpt(self):   #wing for optimizer
        return self.getWing(self.span/self.n * np.ones ,self.chord , self.sweep, self.dihedral, self.twist)
    
    def getWingEval(self,res):  #acutall wing
        span = res.value(self.span)
        chord = res.value(self.chord)
        sweep = res.value(self.sweep)
        dihedral = res.value(self.dihedral)
        twist = res.value(self.twist)
        return self.getWing(span,chord,sweep,dihedral,twist)

    def getMass(self,span,chord,sweep,dihedral,twist, SparFactor, AreaFactor): #estimates the Mass of the Wing as a Spar that has the lenth of the wing and a Area Factor that is wing area*Area Factor

        m = np.sum(span) * SparFactor
        holder = asb.Airplane(name="holder",xyz_ref=[0,0,0],wings=[self.getWing(span,chord,sweep,dihedral,twist)],fuselages=[])
        area = holder.s_ref
        m = m + area * AreaFactor
        return m

    def getMassOpt(self,SparFactor,AreaFactor):
        return self.getMass(self.span ,self.chord , self.sweep, self.dihedral, self.twist, SparFactor, AreaFactor)

    def getMassEval(self,res,SparFactor,AreaFactor):
        #makes adaptation for later evaluation easier
        span = res.value(self.span)
        chord = res.value(self.chord)
        sweep = res.value(self.sweep)
        dihedral = res.value(self.dihedral)
        twist = res.value(self.twist)
        return self.getMass(span,chord,sweep,dihedral,twist, SparFactor, AreaFactor)
        
    
opt = asb.Opti()

#parameters
m0 = 1
v = 10

alpha = 4 #opt.variable(init_guess = 1)
#opt.subject_to(alpha < 5)
#opt.subject_to(alpha > 0)

main_wing = OptimizableWing(opt, name="Main Wing",position = [0, 0, 0], sections = 2, iniSpan = 3, iniCord = 0.3, twistLimit = 2 , sweep = 0, dihedralLimit = 0.1)


#wings = [main_wing.getWing()]
plane = asb.Airplane(name="TestPlane",xyz_ref=[0,0,0],wings=[main_wing.getWingOpt()],fuselages=[])

m = m0 + main_wing.getMassOpt(5,1)
lift = m * 9.81 

vlm = asb.AeroBuildup(
    airplane=plane,
    op_point=asb.OperatingPoint(
        velocity=v,  # m/s
        alpha=alpha,  # degree
    )
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
    ),spanwise_resolution=2,
    chordwise_resolution=8
)
aero = vlm.run()
vlm.draw(show_kwargs=dict(jupyter_backend="static"))

print(res)
