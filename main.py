
import os
from src.util import read_pdpt_csv_to_pickle, ConsoleLogger
from src.pdpt_ini_sol import pdpt_ini_sol
# from pdpt_repari import best_insertion
# from pdpt_rasd import rasd
from pathlib import Path

import argparse

parser = argparse.ArgumentParser(description='read_data read raw data from csv files and store it as dict in pickle files')
parser.add_argument('--verbose', type=int, default=0, help='flag for printing' +\
                                                            '0 - dont print logs' +\
                                                            '>0 - print details of each instance')
parser.add_argument('--datadir', type=str, default='/home/tan/Documents/GitHub/pdpt_2022', help='Directory of PDPT data files')

args = parser.parse_args()

DATA_DIR = args.datadir
VERBOSE = args.verbose

os.environ['DATA_DIR'] = args.datadir

def main():
    greedy_initialization = False

    assert os.path.isdir(DATA_DIR+'/data'), f'Can find data files in {DATA_DIR}/data'

    Path(DATA_DIR+'/out').mkdir(parents=True, exist_ok=True)
    # Start logging all stdout and stderr to out/experiment1/experiment1.log
    logfile = os.path.join(DATA_DIR+'/out', f'pdpt_exp.log')
    ConsoleLogger.start(logfile, mode='w', time_format='[%m-%d %H:%M:%S]')

    # read raw data and convert it to dict and save as pickle files
    # for case_num in (range(1,6,1)):
    # # for case_num in [3]:
    #     with ConsoleLogger.copy_to(os.path.join(DATA_DIR+'/data', f'case{case_num}_read_data.log')):
    #         read_pdpt_csv_to_pickle(case_num, DATA_DIR+'/data', verbose = VERBOSE)

    for case_num in (range(1,6,1)):
    # for case_num in [4,]:
        Path(DATA_DIR+f'/out/case{case_num}_gurobi').mkdir(parents=True, exist_ok=True)
        with ConsoleLogger.copy_to(os.path.join(DATA_DIR+'/out', f'case{case_num}_initSol.log')):

            # construct initial solutions by solving multiple PDOTW 
            pdpt_ini_sol(case_num, DATA_DIR, greedy_initialization, verbose = VERBOSE)

    # repair initial solution through a best_insertion heuristic
    # new_sol = best_insertion(ini_sol)

    # # improve solution through random adaptive saptial decoupling
    # # note, in RASD, each sub-problem is solved using route+schedule decomposition framework (see route_scheudle_decomp.py )
    # final_sol = rasd(new_sol)

    ConsoleLogger.stop()



if __name__ == "__main__":
        

    main()