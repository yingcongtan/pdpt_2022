
import os, pickle
from src.util import read_pdpt_csv_to_pickle, ConsoleLogger
from pdpt_ini_sol import pdpt_ini_sol
# from pdpt_repari import best_insertion
from pdpt_rasd import pdpt_rasd
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
    # for case_num in (range(4,6,1)):
    # # for case_num in [3]:
    #     with ConsoleLogger.copy_to(os.path.join(DATA_DIR+'/data', f'case{case_num}_read_data.log')):
    #         read_pdpt_csv_to_pickle(case_num, DATA_DIR+'/data', verbose = VERBOSE-1)

    # for case_num in (range(4,6,1)):
    # # for case_num in [3,]: 
    #     Path(DATA_DIR+f'/out/case{case_num}_gurobi').mkdir(parents=True, exist_ok=True)
    #     with ConsoleLogger.copy_to(os.path.join(DATA_DIR+'/out', f'case{case_num}_initSol.log')):
    #         # construct initial solutions by solving multiple PDOTW 
    #         pdpt_ini_sol(case_num, DATA_DIR, greedy_initialization, verbose = VERBOSE)

    # repair initial solution through a best_insertion heuristic
    # new_sol = best_insertion(ini_sol)

    # # improve solution through random adaptive saptial decoupling
    # # note, in RASD, each sub-problem is solved using route+schedule decomposition framework (see route_scheudle_decomp.py )
    # path_ = DATA_DIR +'/out'
    # pdpt_ins_filename = os.path.join(DATA_DIR, f'data/case{case_num}.pkl')
    # ini_sol_res_filename = os.path.join(DATA_DIR, f'out/case{case_num}_initSol.pkl')


    for case_num in (range(1,6,1)):
        with ConsoleLogger.copy_to(os.path.join(DATA_DIR+'/out', f'case{case_num}_rasd.log')):
            pdpt_rasd(DATA_DIR, case_num, verbose = 0)

    # pdpt_sol_res_filename  = os.path.join(DATA_DIR, f'out/case{case_num}_finalSol.pkl')

    # with open(pdpt_sol_res_filename, 'wb') as f:
    #     pickle.dump(pdpt_sol, f)
    ConsoleLogger.stop()



if __name__ == "__main__":
        

    main()