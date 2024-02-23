from ament_index_python.packages import get_package_share_directory

class Data():
    def __init__(self):
        package_share_directory = get_package_share_directory('car_demo')
        self.team_code = "" ## change to your team code
        self.submit_choice = False ## change to your wanted option [True/False]
        self.solution_file_path = package_share_directory + "/scripts/solution.py"
        print(self.solution_file_path)
        '''-----DO NOT CHANGE-----'''
        #shutil.make_archive('solution_scripts', 'zip', './src/machathon5.00/car_demo', 'scripts')
        self.solution_file = open(self.solution_file_path, 'r').read()