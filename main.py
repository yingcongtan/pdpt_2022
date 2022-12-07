
import os, time
import pickle
from src.util import read_pdpt_csv_to_pickle, ConsoleLogger
from pdpt_ini_sol import pdpt_ini_sol
from pdpt_repair import pdpt_sol_repair
from pdpt_rasd import pdpt_rasd
from lns_tvopdpt import lns_tvopdpt
from pathlib import Path

import argparse

parser = argparse.ArgumentParser(description='read_data read raw data from csv files and store it as dict in pickle files')
parser.add_argument('--verbose', type=int, default=0, help='flag for printing' +\
                                                            '0 - dont print logs' +\
                                                            '>0 - print details of each instance')
parser.add_argument('--datadir', type=str, default='/home/tan/Documents/GitHub/pdpt_2022', help='Directory of PDPT data files')
parser.add_argument('--timelimit', type=int, default=1800, help='Total runtime limit')

args = parser.parse_args()

DATA_DIR = args.datadir
VERBOSE = args.verbose
RUNTIME_LIMIT = args.timelimit

os.environ['DATA_DIR'] = args.datadir

def main():
    greedy_initialization = False
    optimize_pdotw_route = True
    remaining_time = RUNTIME_LIMIT

    assert os.path.isdir(DATA_DIR+'/data'), f'Can find data files in {DATA_DIR}/data'

    Path(DATA_DIR+'/out').mkdir(parents=True, exist_ok=True)
    # Start logging all stdout and stderr to out/experiment1/experiment1.log
    logfile = os.path.join(DATA_DIR+'/out', f'pdpt_exp.log')
    ConsoleLogger.start(logfile, mode='w', time_format='[%m-%d %H:%M:%S]')

    # read raw data and convert it to dict and save as pickle files
    # for case_num in (range(1,6,1)):
    # # for case_num in [3]:
    #     with ConsoleLogger.copy_to(os.path.join(DATA_DIR+'/data', f'case{case_num}_read_data.log')):
    #         read_pdpt_csv_to_pickle(case_num, DATA_DIR+'/data', verbose = VERBOSE-1)

    for case_num in (range(1,6,1)):
    # for case_num in [1,]:
        # curr_time = time.time()
        print(f'\n______________CASE {case_num}_________________')
        Path(DATA_DIR+f'/out/iniSol/case{case_num}_gurobi').mkdir(parents=True, exist_ok=True)
        with ConsoleLogger.copy_to(os.path.join(DATA_DIR, 'out','iniSol', f'case{case_num}_iniSol.log')):
            # construct initial solutions by solving multiple PDOTW 
            pdpt_ini_sol(case_num, DATA_DIR, greedy_initialization, optimize_pdotw_route, verbose = VERBOSE)
            
        # remaining_time -= time.time()-curr_time
        # curr_time = time.time()
    
        # if remaining_time<0:
        #     print('===== reach runtime limit, exit')
        #     return None
    for case_num in range(1,6,1):
    # for case_num in [3]:
        print(f'\nSTART LNS_TVOPDPT on  case {case_num}======================')
        with ConsoleLogger.copy_to(os.path.join(DATA_DIR, 'out','impSol_tvopdpt', f'case{case_num}.log')):

            lns_tvopdpt(case_num)

    # repair initial solution through a best_insertion heuristic
    # for case_num in [1,]: 
    #     curr_time = time.time()
    #     print(f'\n______________CASE {case_num}_________________')
    #     Path(os.path.join(DATA_DIR, 'out', 'repSol')).mkdir(parents=True, exist_ok=True)
    #     with ConsoleLogger.copy_to(os.path.join(DATA_DIR, 'out','repSol', f'case{case_num}_repSol.log')):
    #         # construct initial solutions by solving multiple PDOTW 
    #         pdpt_sol_repair(case_num, DATA_DIR, verbose = VERBOSE)


    # # improve solution through random adaptive saptial decoupling
    # # note, in RASD, each sub-problem is solved using route+schedule decomposition framework (see route_scheudle_decomp.py )
    # path_ = DATA_DIR +'/out'
    # pdpt_ins_filename = os.path.join(DATA_DIR, f'data/case{case_num}.pkl')
    # ini_sol_res_filename = os.path.join(DATA_DIR, f'out/case{case_num}_initSol.pkl')


    # for case_num in (range(1,6,1)):
    # for case_num in [1,]: 
    #     print(f'\n______________CASE {case_num}_________________')
    #     Path(DATA_DIR+f'/out/impSol/case{case_num}_gurobi').mkdir(parents=True, exist_ok=True)
    #     with ConsoleLogger.copy_to(os.path.join(DATA_DIR, 'out','impSol', f'case{case_num}_iniSol.log')):
    #         pdpt_rasd(DATA_DIR, case_num, num_iteration = 1, verbose = 0)

    ConsoleLogger.stop()



if __name__ == "__main__":
        

    main()