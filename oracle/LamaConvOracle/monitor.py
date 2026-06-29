#Python monitor generated using LamaConv
#This file stores the monitor that is used

class Monitor:
  def __init__(self, callback):
    self.state = "q1"
    self.output = None
    self.callback = callback

  
  
  def transit_to_q1(self):
    self.state = "q1"
    self.output = None
    self.callback(self.output)
  
  
  def transit_to_q0(self):
    self.state = "q0"
    self.output = False
    self.callback(self.output)
  

  
  def transit(self, assignment):


    if self.state == "q0":
      self.transit_to_q0()
      return



    if self.state == "q1":
      if (not assignment["hello"]):
        self.transit_to_q0()
      if (assignment["hello"]):
        self.transit_to_q1()
      return


