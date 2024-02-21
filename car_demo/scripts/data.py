
class Data():
    def __init__(self):
        self.team_code = "HH" ## change to your team code
        self.submit_choice = False ## change to your wanted option [True/False]
        
        '''-----DO NOT CHANGE-----'''
        #shutil.make_archive('solution_scripts', 'zip', './src/machathon5.00/car_demo', 'scripts')
        self.solution_file = open('./src/machathon5-judge/car_demo/scripts/solution.py', 'r').read()