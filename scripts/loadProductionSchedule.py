import openpyxl


def load_production_plan():
    more_products = True
    incrementer = 3

    production_list = []

    while more_products:
        if sheet_one.cell(row=incrementer, column=1).value:
            production_list.append(sheet_one.cell(row=incrementer, column=1).value)
        else:
            more_products = False

        incrementer += 1

    return production_list


workbook = openpyxl.load_workbook('/home/sev-ros/catkin_ws/src/beginner_tutorials/scripts/Daily_Production_Plan.xlsx')
type(workbook)

# To get the names of the sheets in the file use:
# print(workbook.get_sheet_names())

sheet_one = workbook.get_sheet_by_name('Plan')

if __name__ == "__main__":
    more_to_print = True
    i = 3

    while more_to_print:
        if sheet_one.cell(row=i, column=1).value:
            print(sheet_one.cell(row=i, column=1).value)
        else:
            more_to_print = False

        i += 1
