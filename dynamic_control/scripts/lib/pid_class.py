'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''

from __future__ import division
 

##### Classe PID ######
class PID:
    def __init__(self, P, I, D, Tz, etol):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Ts = Tz
        self.Integrator_min = -20.0 
        self.Integrator_max = 20.0
        self.Derivator = 0.0
        self.Integrator = 0.0

    	self.alpha = 0.6
    	self.control = 0.0

        self.set_point = 0.0
        self.error = 0.0
        self.errorant = 0.0
        self.errortolerance = etol

#### Atualizar o valor de controle com base no sinal de entrada ####
    def update(self,reference,feedback):
        self.error = reference - feedback
        if (abs(self.error) < self.errortolerance):
            self.error = 0
        
        ### Proporcional ###
        self.proportional = self.Kp * self.error
        ### Derivativo #####
        self.derivative = self.Kd * (self.error - self.Derivator)/self.Ts
        self.Derivator = self.error  #erro anterior
        ### Integral #######
        self.Integrator = self.Integrator + ((self.errorant + self.error)/2)*self.Ts
        #### saturacao integrador
        # if self.Integrator > self.Integrator_max:
        #     self.Integrator = self.Integrator_max
        # elif self.Integrator < self.Integrator_min:
        #     self.Integrator = self.Integrator_min
        self.errorant = self.error
        self.integral = self.Integrator * self.Ki

        PID = self.proportional + self.integral + self.derivative
        
        #### Saturacao ######
        if( reference == 0):
            PID = 0.0
            self.Integrator = 0.0
        elif(PID >= 2*1256):
            PID = 2*1256
            self.Integrator = self.Integrator - ((self.errorant + self.error)/2)*self.Ts
        elif(PID <= -2*1256):
            PID = -2*1256
            self.Integrator = self.Integrator - ((self.errorant + self.error)/2)*self.Ts

        # Filter	
        self.control = (1-self.alpha)*self.control + self.alpha*PID
        # self.control = PID
		

        return self.control

    
####### Retorna o Erro ###########
    def getError(self):
        return self.error
####### Retornar o Integrador ####
    def getIntegrator(self):
        return self.Integrator
####### Retornar o Derivador #####
    def getDerivator(self):
        return self.Derivator
