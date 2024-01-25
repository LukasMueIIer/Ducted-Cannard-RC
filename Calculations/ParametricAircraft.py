import aerosandbox as asb
import aerosandbox.numpy as np

class OptimizableWing:  #a subclass that makes optimization and mass management easier
    def __init__(self, opt, name="Wing",position = [0, 0, 0], sections = 10, iniSpan = 2, iniCord = 0.5, twistLimit = 0):

        self.position = position
        self.name = name
        self.n = sections

        #set initial guess of coordinates
        self.X = opt.variable(init_guess = np.zeros(sections) + np.ones(sections) * position[0])                  
        self.Y = opt.variable(init_guess = np.linspace(0,iniSpan,sections))
        self.Z = opt.variable(init_guess = np.zeros(sections) + np.ones(sections) * position[2])
        self.C = opt.variable(init_guess = np.ones(sections))
        self.twist = opt.variable(init_guess = np.zeros(sections))

        #add sensical constraints
        for i in range(0,sections):
            if(i == 0):
                opt.subject_to(self.X[0] == position[0]) #force first point to be at origin
                opt.subject_to(self.Y[0] == 0)           #zero cause we want symmetrie
                opt.subject_to(self.Z[0] == position[2]) 
            else:
                opt.subject_to(self.Y[i] >= self.Y[i - 1])  #Xsec must be farter out than the one before
                opt.subject_to(self.C[i] < self.C[i - 1])  #chord can only decrease
            opt.subject_to(np.abs(self.twist[i]) < twistLimit)

        opt.subject_to(self.C[-1] > 0.01) #all cs must be larger than 0 (with increase condition that is true if last one is bigger 0)


        #force no dihedral
        opt.subject_to(self.Z == np.ones(sections) * position[2])
        
        #Airfoil Shapes
        self.XSec = []  
        for i in range(0,sections):
            self.XSec.append(asb.Airfoil("sd7037"))

    def getWing(self,X,Y,Z,C,twist):  #Returns an asb wing object to attach to an aircraft
        n = self.n
        Xsec = []
        for i in range(0,n):
            _x = [X[i],Y[i],Z[i]]
            _c = C[i]
            _t = twist[i]
            _foil = self.XSec[i]
            Xsec.append(asb.WingXSec(xyz_le=_x,chord=_c,twist=_t,airfoil=_foil))

        #create main wing
        wing = asb.Wing(name=self.name,symmetric=True,xsecs=Xsec)
        self.asbWing = wing
        return wing

    def getWingOpt(self):   #wing for optimizer
        return self.getWing(self.X ,self.Y , self.Z, self.C, self.twist)
    
    def getWingEval(self,res):  #acutall wing
        X = res.value(self.X)
        Y = res.value(self.Y)
        Z = res.value(self.Z)
        C = res.value(self.C)
        twist = res.value(self.twist)
        return self.getWing(X,Y,Z,C,twist)

     


    def getMass(self,X ,Y , Z, C, SparFactor, AreaFactor): #estimates the Mass of the Wing as a Spar that has the lenth of the wing and a Area Factor that is wing area*Area Factor

        #makes adaptation for later evaluation easier
        X = self.X
        Y = self.Y
        Z = self.Z
        C = self.C

        m = 0
    
        n = self.n
        for i in range(0,n-1):
            lSpar = np.sqrt((X[i+1] - X[i])**2 + (Y[i+1] - Y[i])**2 +  (Z[i+1] - Z[i])**2)
            m = m + lSpar * SparFactor
        return m

    def getMassOpt(self,SparFactor,AreaFactor):
        return self.getMass(self.X ,self.Y , self.Z, self.C, SparFactor, AreaFactor)

    def getMassEval(self,res,SparFactor,AreaFactor):
        #makes adaptation for later evaluation easier
        X = res.value(self.X)
        Y = res.value(self.Y)
        Z = res.value(self.Z)
        C = res.value(self.C)
        return self.getMass(X ,Y , Z, C, SparFactor, AreaFactor)
        
    



opt = asb.Opti()

#parameters
m0 = 1
v = 10

alpha = 4 #opt.variable(init_guess = 1)
#opt.subject_to(alpha < 5)
#opt.subject_to(alpha > 0)

main_wing = OptimizableWing(opt, name="Main Wing",position = [0, 0, 0], sections = 5, iniSpan = 1, iniCord = 0.1, twistLimit = 2)


#wings = [main_wing.getWing()]
plane = asb.Airplane(name="TestPlane",xyz_ref=[0,0,0],wings=[main_wing.getWingOpt()],fuselages=[])

m = m0 + main_wing.getMassOpt(2,1) + plane.s_ref
lift = m * 9.81 

#vlm = asb.VortexLatticeMethod(
  #  airplane=plane,
   # op_point=asb.OperatingPoint(
   #     velocity=v,  # m/s
  #      alpha=alpha,  # degree
  #  ),spanwise_resolution=1,
  #  chordwise_resolution=8
#)

#aero = vlm.run()

aero = asb.AeroBuildup(
    airplane=plane,
    op_point=asb.OperatingPoint(
        velocity=v,
        alpha=alpha,
        beta=0
    ),
).run()

opt.subject_to(aero["L"] == lift)
opt.minimize(aero["D"])

res = opt.solve()

plane = asb.Airplane(name="ResultPlane",xyz_ref=[0,0,0],wings=[main_wing.getWingEval(res)],fuselages=[])

plane.draw_three_view()

print(res)
