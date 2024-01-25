#Aircraft optimization


#TODO:
# Estimation of Aerodynamic Performance
# Weight Estimation Of Systems
# Power Consumption Estimation of Components
# Stability Analysis -> Maybe Couple OpenVSP and Aerotoolbox? (but not differentiable so mehhh)

##Design Parameters



##Speed Requirements
vStal = 5 #m/s max staal speed -> I wana land at that speed

##Aerodynamic Requirements
#We want a Airfoil we can fly to a CL = 0
#Bucket with size 0.3 around vMax (full power straight level flight)
#will need a Cd Estimate at that Cl -> will obain iteratively 
#iteration will be, create design -> optimize fitting airfoil -> get that Cd at target Cl -> repeat
#ClMax so that we can fly at vStal
#It would be good if we could assume CD of Cl maxSpeed based on ClMax
ClMax = 1.5 #Max CL were willing to force in our optimization -> will force a Wing Size upon us

#Aerodynamic Performance
CD0Fuselage = #CD0 of Fuselage from OpenVSP calculation

##Mass Buildup

#Fixed Mass (independent of optimization)
mBattery = #kg -> datasheet
mMotors = #kg -> datasheet
mServos  = #kg -> datasheet
mReciever = #kg -> datasheet
mFuselage = #kg -> encapsulating shape and estimation from OpenVSP and Prusa Slicer
mProp = #kg -> datasheet
mWiring = #kg -> rough estimation, maybe 1m of wire 
mDuct = #kg -> from OpenVSP, estimation

#Variable Masses (change with design parameters) (mainly Wings)
#Soooo the structure assumption is we have one Spar and a Skin
#Ill create multiple wings(variable spann and Chord) and this will alow us to estimate the Mass




