import aerosandbox as asb
import aerosandbox.numpy as np

class OptimizableWing:  #a subclass that makes optimization and mass management easier
    def __init__(self, opt, name="Wing",position = [0, 0, 0], sections = 10, iniSpan = 2, iniCord = 0.5):

        self.position = position
        self.name = name

        #set initial guess of coordinates
        self.X = opt.variable(init_guess = np.zeros(sections))                  
        self.Y = opt.variable(init_guess = np.linspace(0,iniSpan,sections))
        self.Z = opt.variable(init_guess = np.zeros(sections))
        self.C = opt.variable(init_guess = np.ones(sections))
        self.twist = opt.variable(init_guess = np.zeros(sections))

        #add sensical constraints
        opt.subject_to(self.X[0] == 0) #force first point to be at origin
        opt.subject_to(self.Y[0] == 0) 
        opt.subject_to(self.Z[0] == 0) 

        for i in range(1,sections):
            opt.subject_to(self.Y[i] >= self.Y[i - 1])  #Xsec must be farter out than the one before
            opt.subject_to(self.C[i] >= self.C[i - 1])  #chord can only decrease

        opt.subject_to(self.C[-1] > 0) #all cs must be larger than 0 (with increase condition that is true if last one is bigger 0)
        
        #Airfoil Shapes
        self.XSec = []  
        for i in range(0,sections):
            self.XSec.append(asb.Airfoil("sd7037"))


    def getWing(self):  #Returns an asb wing object to attach to an aircraft
        n = len(self.X)
        Xsec = []
        for i in range(0,n):
            _x = [self.X[i],self.Y[i],self.Z[i]]
            _c = self.C[i]
            _t = self.twist[i]
            _foil = self.XSec[i]
            Xsec.append(asb.WingXSec(xyz_le=_x,chord=_c,twist=_t,airfoil=_foil))

        #create main wing
        wing = asb.Wing(name=self.name,symmetric=True,xsecs=[mW_X_Root,mW_X_1,mW_X_2])
        wing = main_wing.translate(self.position)
        return wing

    def getMass(SparFactor,AreaFactor): #estimates the Mass of the Wing as a Spar that has the lenth of the wing and a Area Factor that is wing area*Area Factor

        m = 0
        m = m + SparFactor * self.Y[-1] #Spar is as long as final xSec position
        n = len(self.X)
        for i in range(0,n-1):
            area = 0.5 * (self.C[i] + self.C[i+1]) * (self.X[i+1] - self.X[i])
            m = m + area * AreaFactor
        return m



opt = asb.Opti()

#parameters
m0 = 1
v = 10

main_wing = OptimizableWing(opt, name="Main Wing",position = [0, 0, 0], sections = 4, iniSpan = 2, iniCord = 0.5)

m = m0 + main_wing.getMass(1,1)
lift = m * 9.81 
