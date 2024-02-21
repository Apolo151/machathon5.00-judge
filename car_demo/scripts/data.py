import shutil

class Data():
    def __init__(self):
        self.team_code = "" ## change to your team code
        self.submit_choice = False ## change to your wanted option [True/False]
        
        '''-----DO NOT CHANGE-----'''
        shutil.make_archive('solution_scripts', 'zip', './src/machathon5.00/car_demo', 'scripts')
        self.solution = "solution_scripts.zip"