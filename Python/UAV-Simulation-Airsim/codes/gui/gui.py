import tkinter as tk
from algos.algo1 import *
from algos.algo2 import *
from algos.algo3 import *
from genjson.genjson import *

class InputApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry("500x300")
        self.root.title("Algorithm Selector")
        self.input_filename = tk.StringVar()
        self.input_radious = tk.StringVar()
        self.input_size = tk.StringVar()
        self.input_fps = tk.StringVar()
        self.sSize = 0
        self.index = 0
        self.input_boxes = [
            self.create_input_box1(self.input_filename, self.input_radious, self.input_size),
            self.create_input_box2()            
        ]
        self.back_button = tk.Button(self.root, text="\u2190", background = "orange", width=10,command=self.go_back)
        self.forward_button = tk.Button(self.root, text="\u2192", background = "orange", width=10, command=self.go_forward)    
        self.algo_num = 0
        self.show_input_box()
        
    def create_input_box1(self, filename, radious, size):
        label_filename = tk.Label(self.root, text = 'File Name')
        filename.set("")
        entry1 = tk.Entry(self.root, exportselection=0, width=20, textvariable=filename)
        label_radious = tk.Label(self.root, text = 'Radius of Hexagon')
        radious.set("")
        entry2 = tk.Entry(self.root, exportselection=0, width=20, textvariable=radious)  
        label_size = tk.Label(self.root, text = 'Swarm Size')
        size.set("")
        entry3 = tk.Entry(self.root, exportselection=0, width=20, textvariable=size)
        button1 = tk.Button(self.root, text= "Submit",width= 20, background = "light blue", command=self.submitArgs)    
        exit_button =  tk.Button(self.root, text="Exit",  background = "yellow", width=10,command=self.exit_app)                   
        return (label_filename, entry1, label_radious, entry2, label_size, entry3, button1, exit_button)  
        
    def create_input_box2(self):
        b1 = tk.Button(self.root, text="Algorithm1", background = "light blue", width=10, command=self.on_button_click_Algo1)
        b2 = tk.Button(self.root, text="Algorithm2", background = "light blue", width=10, command=self.on_button_click_Algo2)
        b3 = tk.Button(self.root, text="Algorithm3", background = "light blue", width=10, command=self.on_button_click_Algo3)
        b4 = tk.Button(self.root, text="Algorithm4", background = "light blue", width=10, command=self.on_button_click_Algo4)
        start_button = tk.Button(self.root, text="Start", background = "green", width=10, command=self.start_click)
        stop_button =  tk.Button(self.root, text="Stop",  background = "red", width=10,command=self.stop_click)
        exit_button =  tk.Button(self.root, text="Exit",  background = "yellow", width=10,command=self.exit_app)           
        return (b1, b2, b3, b4, start_button, stop_button, exit_button)
        
    def submitArgs(self):        
        self.go_forward()
        print("The filename is : " + self.input_filename.get())
        print("The radious is : " + self.input_radious.get())
        print("The swarm size is : " + self.input_size.get())
        self.sSize = int(self.input_size.get())
        if self.sSize < 2:
            self.sSize = 2        
        self.argsPoly = {"filename":self.input_filename.get(), "radious":int(self.input_radious.get()), "size":self.sSize}  
        runGenJson(self.sSize)        

    def on_button_click_Algo1(self):
        self.winAlgo1 = tk.Toplevel(self.root)
        self.winAlgo1.title("Algorithm 1")
        self.winAlgo1.geometry("+{}+{}".format(self.root.winfo_x()+self.root.winfo_width(), self.root.winfo_y()))
        title_width = self.winAlgo1.winfo_reqwidth()
        self.winAlgo1.geometry("{}x250".format(min(title_width*2, 300)))        
        label1 = tk.Label(self.winAlgo1, text = 'Survey Size').pack()
        self.entry1Algo1 = tk.Entry(self.winAlgo1, exportselection=0, width=20)
        self.entry1Algo1.pack()
        label2 = tk.Label(self.winAlgo1, text = 'Stripe Width').pack()
        self.entry2Algo1 = tk.Entry(self.winAlgo1, exportselection=0, width=20)
        self.entry2Algo1.pack()  
        label3 = tk.Label(self.winAlgo1, text = 'Altitude').pack()
        self.entry3Algo1 = tk.Entry(self.winAlgo1, exportselection=0, width=20)
        self.entry3Algo1.pack()
        label4 = tk.Label(self.winAlgo1, text = 'Speed').pack()
        self.entry4Algo1 = tk.Entry(self.winAlgo1, exportselection=0, width=20)
        self.entry4Algo1.pack()            
        button1 = tk.Button(self.winAlgo1, text= "Submit",width= 20, background = "light blue", command=self.submitAlgo1).pack(padx=10, pady=10)  
  
        
    def submitAlgo1(self):
        print("The survey size is : " + self.entry1Algo1.get())
        print("The survey stripewidth is : " + self.entry2Algo1.get())
        print("The survey altitude is : " + self.entry3Algo1.get())
        print("The survey speed is : " + self.entry4Algo1.get())
        self.algo_num = 1
        print("Algorithm {} is chosen !".format(self.algo_num))
        self.argsSurvey = {"size":float(self.entry1Algo1.get()), "stripewidth":float(self.entry2Algo1.get()), "altitude":float(self.entry3Algo1.get()), "speed":float(self.entry4Algo1.get())}   
        self.winAlgo1.destroy()
        
    def on_button_click_Algo2(self):
        self.winAlgo2 = tk.Toplevel(self.root)
        self.winAlgo2.title("Algorithm 2")
        self.winAlgo2.geometry("+{}+{}".format(self.root.winfo_x()+self.root.winfo_width(), self.root.winfo_y()))
        title_width = self.winAlgo2.winfo_reqwidth()
        self.winAlgo2.geometry("{}x250".format(min(title_width*2, 300)))    
        label1 = tk.Label(self.winAlgo2, text = 'FPS').pack()
        self.entry1Algo2 = tk.Entry(self.winAlgo2, exportselection=0, width=20)
        self.entry1Algo2.pack()           
        button1 = tk.Button(self.winAlgo2, text= "Submit",width= 20, background = "light blue", command=self.submitAlgo2).pack()  

    def submitAlgo2(self):
        print("The FPS is : " + self.entry1Algo2.get())  
        self.input_fps = self.entry1Algo2.get()
        self.algo_num = 2
        print("Algorithm {} is chosen !".format(self.algo_num))
        self.winAlgo2.destroy()

    def on_button_click_Algo3(self):
        self.winAlgo3 = tk.Toplevel(self.root)
        self.winAlgo3.title("Algorithm 3")
        self.winAlgo3.geometry("+{}+{}".format(self.root.winfo_x()+self.root.winfo_width(), self.root.winfo_y()))
        title_width = self.winAlgo3.winfo_reqwidth()
        self.winAlgo3.geometry("{}x250".format(min(title_width*2, 300)))    
        label1 = tk.Label(self.winAlgo3, text = 'FPS').pack()
        self.entry1Algo3 = tk.Entry(self.winAlgo3, exportselection=0, width=20)
        self.entry1Algo3.pack()           
        button1 = tk.Button(self.winAlgo3, text= "Submit",width= 20, background = "light blue", command=self.submitAlgo3).pack()  
        
    def submitAlgo3(self):
        print("The FPS is : " + self.entry1Algo3.get())  
        self.input_fps = self.entry1Algo3.get()
        self.algo_num = 3
        print("Algorithm {} is chosen !".format(self.algo_num))
        self.winAlgo3.destroy()    

    def on_button_click_Algo4(self):
        self.winAlgo4 = tk.Toplevel(self.root)
        self.winAlgo4.title("Algorithm 4")
        self.winAlgo4.geometry("+{}+{}".format(self.root.winfo_x()+self.root.winfo_width(), self.root.winfo_y()))
        title_width = self.winAlgo4.winfo_reqwidth()
        self.winAlgo4.geometry("{}x250".format(min(title_width*2, 300)))   
        label1 = tk.Label(self.winAlgo4, text = 'Survey Size').pack()
        self.entry1Algo4 = tk.Entry(self.winAlgo4, exportselection=0, width=20)
        self.entry1Algo4.pack()
        label2 = tk.Label(self.winAlgo4, text = 'Stripe Width').pack()
        self.entry2Algo4 = tk.Entry(self.winAlgo4, exportselection=0, width=20)
        self.entry2Algo4.pack()  
        label3 = tk.Label(self.winAlgo4, text = 'Altitude').pack()
        self.entry3Algo4 = tk.Entry(self.winAlgo4, exportselection=0, width=20)
        self.entry3Algo4.pack()
        label4 = tk.Label(self.winAlgo4, text = 'Speed').pack()
        self.entry4Algo4 = tk.Entry(self.winAlgo4, exportselection=0, width=20)
        self.entry4Algo4.pack()            
        button1 = tk.Button(self.winAlgo4, text= "Submit",width= 20, background = "light blue", command=self.submitAlgo4).pack()  

        
    def submitAlgo4(self):
        print("The survey size is : " + self.entry1Algo4.get())
        print("The survey stripewidth is : " + self.entry2Algo4.get())
        print("The survey altitude is : " + self.entry3Algo4.get())
        print("The survey speed is : " + self.entry4Algo4.get())
        self.algo_num = 4  
        print("Algorithm {} is chosen !".format(self.algo_num))
        self.argsSurvey = {"size":float(self.entry1Algo4.get()), "stripewidth":float(self.entry2Algo4.get()), "altitude":float(self.entry3Algo4.get()), "speed":float(self.entry4Algo4.get())}   
        self.winAlgo4.destroy()        
        
    def show_fb_bb(self):
        self.back_button.pack()
        self.forward_button.pack() 
        
    def show_input_box(self):
        for widget in self.root.winfo_children():
            widget.pack_forget()
        current_box = self.input_boxes[self.index]
        for widget in current_box[:-2]:
            widget.pack()
        current_box[-2].pack(padx=10, pady=10)            
        current_box[-1].pack()
            
    def go_back(self):
        if self.index > 0:
            self.index -= 1
            self.show_input_box()
            self.show_fb_bb()

    def go_forward(self):
        if self.index < len(self.input_boxes) - 1:
            self.index += 1
            self.show_input_box()
            self.show_fb_bb()

    def run(self):
        self.root.mainloop()        
    
    def exit_app(self): 
        self.root.destroy()
        
    def start_click(self):
        if self.algo_num == 1:    
            runAlgo1WithThreads(self.argsSurvey, self.sSize)
        elif self.algo_num == 2:
            runPoly2(self.argsPoly, float(self.input_fps))           
        elif self.algo_num == 3:
            runPoly3(self.argsPoly, float(self.input_fps))
        elif self.algo_num == 4:
            runAlgo1WithThreads(self.argsSurvey, self.sSize)

    def stop_click(self):
        if self.algo_num == 1:
            pass
        elif self.algo_num == 2:
            pass
        elif self.algo_num == 3:
            pass        
        elif self.algo_num == 4:
            pass