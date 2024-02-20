import shutil

class Data():
    def __init__(self):
        self.team_name = "" ## change to team name
        self.team_code = "" ## change to team code
        self.submit_choice = False ## change to your option
        
        shutil.make_archive('solution_scripts', 'zip', './src/machathon5.00/car_demo', 'scripts')
        self.solution = "solution_scripts.zip"