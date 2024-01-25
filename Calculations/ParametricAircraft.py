import aerosandbox as asb
import aerosandbox.numpy as np

class OptimizableWing:  #a subclass that makes optimization and mass management easier
    def __init__(self, opt, position = [0, 0, 0], sections = 10, iniSpan = 2, iniCord = 0.5):

        self.position = position

        #set initial guess of coordinates
        self.X = opt.variable(init_guess = np.zeros(sections))                  
        self.Y = opt.variable(init_guess = np.linspace(0,iniSpan,sections))
        self.Z = opt.variable(init_guess = np.zeros(sections))
        self.C = opt.variable(init_guess = np.ones(sections))

        #Airfoil Shapes
        self.XSec = []  
        for i in range(0,sections):
            self.XSec.append(asb.Airfoil("sd7037"))


    def getWing(self):  #Returns an asb wing object to attach to an aircraft
        n = len(self.X)
        #create main wing
        wing_airfoil = asb.Airfoil("sd7037")
        mW_X_Root = asb.WingXSec(xyz_le=[0,0,0],chord=c0,twist=2,airfoil=wing_airfoil)
        mW_X_1 = asb.WingXSec(xyz_le=x1,chord=c1, twist=0, airfoil=wing_airfoil)
        mW_X_2 = asb.WingXSec(xyz_le=x2,chord=c2,twist=0, airfoil=wing_airfoil)
        main_wing = asb.Wing(name="Main Wing",symmetric=True,xsecs=[mW_X_Root,mW_X_1,mW_X_2])


class DuctedCannard:
    def __init__(self):
        self.opt = asb.Opti() #our optimizer environment
        self.wings = []
        self.fuselage = []
    def AddWing(self, sections = 10, iniSpan = 2, iniCord = 0.5): #creates a main wing for optimization
        