#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import loadProductionSchedule
try:
    import tkinter
except ImportError:  # python 2
    import Tkinter as tkinter


""" Takes in a number string and returns the coresponding part and color"""
def convert_to_part_and_color(number_message):
	part_and_color = ['EMPTY', '#e0e0e0']
	if number_message == '1':
		part_and_color[0] = 'C1'
		part_and_color[1] = '#ffb3ba'
	elif number_message == '2':
		part_and_color[0] = 'C2'
		part_and_color[1] = '#ffdfba'
	elif number_message == '3':
		part_and_color[0] = 'C3'
		part_and_color[1] = '#ffffba'
	elif number_message == '4':
		part_and_color[0] = 'C4'
		part_and_color[1] = '#baffc9'
	elif number_message == '5':
		part_and_color[0] = 'C5'
		part_and_color[1] = '#bae1ff'
	elif number_message == '6':
		part_and_color[0] = 'C6'
		part_and_color[1] = '#edbff0'

	return part_and_color


""" Takes in a slot number and a part number and change the interface"""
def change_robot_slots(slot, part_number):
	# Converts the part number to a part name and color
	part_and_color = convert_to_part_and_color(part_number)

	# Changes the interface for the desired slot
	if slot == 1:
		robot_slot_1["text"] = part_and_color[0]
		robot_slot_1["bg"] = part_and_color[1]
	elif slot == 2:
		robot_slot_2["text"] = part_and_color[0]
		robot_slot_2["bg"] = part_and_color[1]


""" Takes in a slot number and a part number and change the interface"""
def change_assembly_slots(slot, part_number):
	# Converts the part number to a part name and color
	part_and_color = convert_to_part_and_color(part_number)

	# Changes the interface for the desired slot
	if slot == 1:
		assembly_slot_1["text"] = part_and_color[0]
		assembly_slot_1["bg"] = part_and_color[1]
	elif slot == 2:
		assembly_slot_2["text"] = part_and_color[0]
		assembly_slot_2["bg"] = part_and_color[1]
	elif slot == 3:
		assembly_slot_3["text"] = part_and_color[0]
		assembly_slot_3["bg"] = part_and_color[1]
	elif slot == 4:
		assembly_slot_4["text"] = part_and_color[0]
		assembly_slot_4["bg"] = part_and_color[1]
	elif slot == 5:
		assembly_slot_5["text"] = part_and_color[0]
		assembly_slot_5["bg"] = part_and_color[1]
	elif slot == 6:
		assembly_slot_6["text"] = part_and_color[0]
		assembly_slot_6["bg"] = part_and_color[1]


""" Function that takes in published data
	and splits it into pieces for the robot inventory interface"""
def robot_callback(data):
	# Saves the data in its string form
	info_gotten = data.data
	# Splits the data string and stores them in a list
	words = info_gotten.split(' ')
	# Calls the change_robot_slots function
	change_robot_slots(1, words[0])
	change_robot_slots(2, words[1])


""" Function that takes in published data 
	and splits it into pieces for the assembly line interface"""
def assembly_callback(data):
	# Saves the data in its string form
	info_gotten = data.data
	# Splits the data string and stores them in a list
	words = info_gotten.split(' ')
	# Calls the change_assembly_slots function
	change_assembly_slots(1, words[0])
	change_assembly_slots(2, words[1])
	change_assembly_slots(3, words[2])
	change_assembly_slots(4, words[3])
	change_assembly_slots(5, words[4])
	change_assembly_slots(6, words[5])


""" Function that initializes this as the interface node
	and subscribes to product_schedule, robot_inventory and assembly_line subjects"""
def listener():
	# Initializes the node as interface
    rospy.init_node('interface', anonymous=True)
    # Subscribes to robot_inventory and assembly_line topics
    rospy.Subscriber('robot_inventory', String, robot_callback)
    rospy.Subscriber('assembly_line', String, assembly_callback)
    """***SHOULD ALSO SUBSCRIBE TO PRODUCTION SCHEDULE***"""

    # For running the tkinter GUI - replaces the rospy.spin function
    main_window.mainloop()


""" CREATING THE MAIN WINDOW FOR THE INTERFACE """

# Creating a main window for the user interface
main_window = tkinter.Tk()

# Defining title of window
main_window.title("Quality Control")
# Defining the geometry of the window and where on screen to place it
main_window.geometry("680x480+100+100")

# Adding padding in x and y for interface window
main_window["padx"] = 8
main_window["pady"] = 8

# Creates a header for the production schedule
heading = tkinter.Label(main_window, text='Daily production schedule', relief='ridge', width=30, bg='#2286c3', fg='#ffffff')
heading.grid(row=0, column=0, columnspan=2, sticky='nsew')

# Creates sub-headers for products and statuses
product_text = tkinter.Label(main_window, text='Product', relief='ridge', width=15, bg='#64b5f6')
product_text.grid(row=1, column=0, columnspan=1, sticky='nsew')
status_text = tkinter.Label(main_window, text='Status', relief='ridge', width=15, bg='#64b5f6')
status_text.grid(row=1, column=1, columnspan=1, sticky='nsew')

# Uses the loadProductionSchedule class to save the production plan as a list
product_list = loadProductionSchedule.load_production_plan()
rospy.loginfo("Loading in production plan..")

# Goes through the list and adds each product to the interface
for i in range(2, (len(product_list))+2):
    tkinter.Label(main_window, text=product_list[i-2], relief='ridge', width=15, bg='#e0e0e0').grid(row=i, column=0, columnspan=1, sticky='nsew')
    tkinter.Label(main_window, text='Waiting', relief='ridge', width=15, bg='#e0e0e0').grid(row=i, column=1, columnspan=1, sticky='nsew')
    #tkinter.Button(main_window, text='Fail', command=lambda: do_stuff("lambda can take parameters"), relief='ridge', bg='red').grid(row=i, column=2, columnspan=1, sticky='nsew')


# Creating robot slots
heading_robot = tkinter.Label(main_window, text='Robot inventory slots', relief='ridge', width=30, bg='#64b5f6')
heading_robot.grid(row=1, column=6, columnspan=2, sticky='nsew')
robot_slot_1 = tkinter.Label(main_window, text='EMPTY', relief='ridge', width=15, bg='#e0e0e0')
robot_slot_1.grid(row=2, column=6, columnspan=1, sticky='nsew')
robot_slot_2 = tkinter.Label(main_window, text='EMPTY', relief='ridge', width=15, bg='#e0e0e0')
robot_slot_2.grid(row=2, column=7, columnspan=1, sticky='nsew')

# Filler to make space between production schedule and slots
filler_slot = tkinter.Label(main_window, text=' ', width=10)
filler_slot.grid(row=0, column=5, columnspan=1, sticky='nsew')

# Creating assembly line slots
heading_assembly = tkinter.Label(main_window, text='Parts in assembly', relief='ridge', width=30, bg='#64b5f6')
heading_assembly.grid(row=5, column=6, columnspan=2, sticky='nsew')
assembly_slot_1 = tkinter.Label(main_window, text='EMPTY', relief='ridge', width=15, bg='#e0e0e0')
assembly_slot_1.grid(row=6, column=6, columnspan=1, sticky='nsew')
assembly_slot_2 = tkinter.Label(main_window, text='EMPTY', relief='ridge', width=15, bg='#e0e0e0')
assembly_slot_2.grid(row=6, column=7, columnspan=1, sticky='nsew')
assembly_slot_3 = tkinter.Label(main_window, text='EMPTY', relief='ridge', width=15, bg='#e0e0e0')
assembly_slot_3.grid(row=7, column=6, columnspan=1, sticky='nsew')
assembly_slot_4 = tkinter.Label(main_window, text='EMPTY', relief='ridge', width=15, bg='#e0e0e0')
assembly_slot_4.grid(row=7, column=7, columnspan=1, sticky='nsew')
assembly_slot_5 = tkinter.Label(main_window, text='EMPTY', relief='ridge', width=15, bg='#e0e0e0')
assembly_slot_5.grid(row=8, column=6, columnspan=1, sticky='nsew')
assembly_slot_6 = tkinter.Label(main_window, text='EMPTY', relief='ridge', width=15, bg='#e0e0e0')
assembly_slot_6.grid(row=8, column=7, columnspan=1, sticky='nsew')

""" END OF CREATING INTERFACE """

# If this is the main file run this code
if __name__ == '__main__':
	# Calls the listener function
    listener()
