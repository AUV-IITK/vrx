import scipy.linalg
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

class GridMap():
    def __init__(self, xlen, ylen, size):
        self.xlen=xlen+2
        self.ylen=ylen+2
        self.size=size
        self.probmap=np.zeros(xlen,ylen)
        self.beamw=2.0
        self.thick=1.0
        self.gridposemap=np.array([np.tile(np.arange(0,self.xlen*self.size, self.size)[:,None],(1,self.ylen)),np.tile((np.arange(0,self.ylen*self.size, self.size)[:,None]).T,(self.xlen,1))])
        self.lidarprob=0.65
        self.ocpro=(self.lidarprob/(1-self.lidarprob))
        self.frpro=-self.ocpro
        
    def Update(self, pose, z):
        delta=self.gridposemap.copy()
        delta[0,:,:]-=pose[0]
        delta[1,:,:]-=pose[1]
        theta_rel=arctan(delta[1,:,:]/delta[0,:,:])-pose[2]
        theta_rel[theta_rel>np.pi]-=2*np.pi
        theta_rel[theta_rel<-np.pi]+=2*np.pi
        r_rel=np.linalg.norm(delta, axis=0)
        
        for zi in z:
            self.probmap[np.abs(theta_rel-zi[1])<=self.beamw/2.0 & r_rel<(zi[0]-self.thick/2.0)]+=self.frpro
            self.probmap[np.abs(theta_rel-zi[1])<=self.beamw/2.0 & np.abs(r_rel-zi[0])<=self.thick/2.0]+=self.ocpro
            