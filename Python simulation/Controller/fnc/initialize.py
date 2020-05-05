import csv
from scipy.interpolate import interp1d
# import numpy as np

# initalize

class centerline_ini:
    def __init__(self,readpath):
        self.course=[]
        self.X=[]
        self.Y=[]
        self.Vx=[]
        self.kap=[]
        self.psi=[]
        self.intp_X=[]
        self.intp_Y=[]
        self.intp_Vx=[]
        self.intp_kap=[]
        self.intp_psi=[]
        self.read(readpath)

        
    def read(self,readpath):
        f=open (readpath,'r')
        f_csv = csv.reader(f)
        for row0 in f_csv:
            row=list(map(lambda x: float(x), row0))
            self.course.append(row[0])
            self.X.append(row[1])
            self.Y.append(row[2])
            self.Vx.append(row[3])
            self.kap.append(row[4])
            self.psi.append(row[5])

        f.close()
        
        self.interpolate()
        
        
    def interpolate(self):
        # self.intp_s=interp1d(self.course,self.s,kind='cubic')
        self.intp_X=interp1d(self.course,self.X,kind='cubic')
        self.intp_Y=interp1d(self.course,self.Y,kind='cubic')
        self.intp_Vx=interp1d(self.course,self.Vx,kind='cubic')
        self.intp_kap=interp1d(self.course,self.kap,kind='cubic')
        self.intp_psi=interp1d(self.course,self.psi,kind='cubic')

        
class reference_ini:
    def __init__(self,readpath):
        self.course=[]
        self.X=[]
        self.Y=[]
        self.Vx=[]
        self.kap=[]
        self.psi=[]
        self.intp_X=[]
        self.intp_Y=[]
        self.intp_Vx=[]
        self.intp_kap=[]
        self.intp_psi=[]
        
        self.Vx_add=0
        self.read(readpath)

        
    def read(self,readpath):
        f=open (readpath,'r')
        f_csv = csv.reader(f)
        for row0 in f_csv:
            row=list(map(lambda x: float(x), row0))
            self.course.append(row[0])
            self.X.append(row[1])
            self.Y.append(row[2])
            self.Vx.append(row[3]+self.Vx_add)
            self.kap.append(row[4])
            self.psi.append(row[5])

        f.close()
        
        self.interpolate()
        
        
    def interpolate(self):
        # self.intp_s=interp1d(self.course,self.s,kind='cubic')
        self.intp_X=interp1d(self.course,self.X,kind='cubic')
        self.intp_Y=interp1d(self.course,self.Y,kind='cubic')
        self.intp_Vx=interp1d(self.course,self.Vx,kind='cubic')
        self.intp_kap=interp1d(self.course,self.kap,kind='cubic')
        self.intp_psi=interp1d(self.course,self.psi,kind='cubic')
        
        
        
        
        

