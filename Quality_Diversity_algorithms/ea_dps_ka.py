#################################################################################################
#                                                                                               #
#                                                                                               #
#                Evolutionary playground: from convergent to divergent search                   #
#                                                                                               #
#                                                                                               #
#################################################################################################
#                                                                                               #
#                                                                                               #
#   Copyright (C) 2020 Stephane Doncieux, Sorbonne Université                                   #
#                                                                                               #
#  This program is free software; you can redistribute it and/or modify it under the terms      #
#  of the GNU General Public License as published by the Free Software Foundation;              #
#  either version 2 of the License, or (at your option) any later version.                      #
#                                                                                               #
#  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;    #
#  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    #
#  See the GNU General Public License for more details.                                         #
#                                                                                               #
#  You should have received a copy of the GNU General Public License along with this program;   #
#  if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,             #
#  Boston, MA 02111-1307 USA                                                                    #
#                                                                                               #
#                                                                                               #
#################################################################################################
#                                                                                               #
# This code allows to run different variants of gradient free direct policy search algorithms   #
# It relies on the DEAP framework to allow an easy exploration of EA components (selection,     #
# mutation, ...), see https://deap.readthedocs.io for more details.                             #
#                                                                                               #
# To use it, set the env_name variable below and launch it with python:                                                        #
#       python3 ea_dps.py                                                                        #
#                                                                                               #
# If you have multiple cores on your computer, consider using scoop, it will parallelize        #
# the run and thus greatly accelerate it:                                                       #
#       python3 -m scoop ea_dps.py                                                               #
#                                                                                               #
#################################################################################################



from deap import *
import numpy as np

import datetime

from deap import algorithms
from deap import base
from deap import creator
from deap import tools

import array
import random
import operator
import os.path

from scoop import futures

from novelty_search import *
import os
from math import cos, sin, pi, sqrt

import grid_management

# Computing segment intersection (from https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/)
def ccw(A,B,C):
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


class Arm:
    def __init__(self, lengths, walls=[]):
        self.n_dofs = len(lengths)
        self.lengths = np.concatenate(([0], lengths))
        self.joint_xy = []
        self.walls=walls

    def fw_kinematics(self, p):
        assert(len(p) == self.n_dofs)
        p = np.append(p, 0)
        self.joint_xy = []
        mat = np.matrix(np.identity(4))
        for i in range(0, self.n_dofs + 1):
            m = [[cos(pi*p[i]), -sin(pi*p[i]), 0, self.lengths[i]],
                 [sin(pi*p[i]),  cos(pi*p[i]), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]
            mat = mat * np.matrix(m)
            v = mat * np.matrix([0, 0, 0, 1]).transpose()
            self.joint_xy += [v[0:2].A.flatten()]
        inters=False
        for ip in range(len(self.joint_xy)-1):
            for iw in range(len(self.walls)):
                inters=intersect(self.joint_xy[ip], self.joint_xy[ip+1], self.walls[iw][0], self.walls[iw][1])
                if inters:
                    break
            if inters:
                break
        return self.joint_xy[self.n_dofs], self.joint_xy, inters


def eval_ka(angles, lengths, target_pos, walls, resdir=None, render=False, dump=False, name=""):
    a=Arm(lengths, walls)
    v, _, inters = a.fw_kinematics(angles)
    dist_ee= -np.linalg.norm(v-target_pos)
    if inters:
        dist_ee=-1000000
        v=[0,0]
    log={
            "pos_end_effector": v,
            "dist_end_effector": dist_ee,
            "collision": inters
        }
    return dist_ee, v, log 

registered_envs={}

gym26={
    'render_mode': 'human'
    }

# all parameters of a run are here, change it or duplicate it to explore different possibilities
# the parameters are grouped into subset of parameters to limite their duplication.
# They are concatenated below to create registered_envs entries.

arm_env1={
    'eval_params': {
        'lengths': [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        'target_pos': [-2, 0.],
        'walls': [
                    [[-1,-1], [-1, 1]],
                    [[-1,-1], [-3, -1]],
                    [[-1,1], [-3, 1]],
                 ]
    },


    'dim_grid': [100, 100],
    'grid_min_v': [-4,-4],
    'grid_max_v': [4,4],
    'goal': [0.6,0.6],

    'watch_max': "dist_end_effector"
}
arm_env1["gen_size"]=len(arm_env1["eval_params"]["lengths"])

ea_generic={
    'min_value': -1, # min genotype value
    'max_value': 1, # max genotype value
    'min_strategy': 0.01, # min value for the mutation
    'max_strategy': 0.2, # max value for the mutation
    'nb_gen': 1000, # number of generations
    'mu': 100, # population size
    'lambda': 200, # number of individuals generated
    'nov_k': 15, # k parameter of novelty search
    'nov_add_strategy': "random", # archive addition strategy (either 'random' or 'novel')
    'nov_lambda': 6, # number of individuals added to the archive
}

ea_random_sampling={
    'min_value': -1, # min genotype value
    'max_value': 1, # max genotype value
    'min_strategy': 0.1, # min value for the mutation
    'max_strategy': 0.5, # max value for the mutation
    'nb_gen': 0, # number of generations
    'mu': 10000, # population size
    'lambda': 200, # number of individuals generated
    'nov_k': 15, # k parameter of novelty search
    'nov_add_strategy': "random", # archive addition strategy (either 'random' or 'novel')
    'nov_lambda': 6, # number of individuals added to the archive
}


ea_NS={
    'selection': 'NS', # can be either NS, FIT or FIT+NS
}

ea_FIT={
    'selection': 'FIT', # can be either NS, FIT or FIT+NS
}

ea_FIT_NS={
    'selection': 'FIT+NS', # can be either NS, FIT or FIT+NS
}



# Arm with NS
registered_envs["arm_NS"]={}
registered_envs["arm_NS"].update(arm_env1)
registered_envs["arm_NS"].update(ea_generic)
registered_envs["arm_NS"].update(ea_NS)

# Arm with FIT
registered_envs["arm_FIT"]={}
registered_envs["arm_FIT"].update(arm_env1)
registered_envs["arm_FIT"].update(ea_generic)
registered_envs["arm_FIT"].update(ea_FIT)

# Arm with FIT+NS
registered_envs["arm_FIT_NS"]={}
registered_envs["arm_FIT_NS"].update(arm_env1)
registered_envs["arm_FIT_NS"].update(ea_generic)
registered_envs["arm_FIT_NS"].update(ea_FIT_NS)

# Arm with RANDOM
registered_envs["arm_RANDOM"]={}
registered_envs["arm_RANDOM"].update(arm_env1)
registered_envs["arm_RANDOM"].update(ea_random_sampling)
registered_envs["arm_RANDOM"].update(ea_FIT)


# change this variable to choose the environment you are interested in 
# (one among the keys of registered_envs)
env_name="arm_FIT"




if (registered_envs[env_name]['selection']=="FIT+NS"):
    creator.create("MyFitness", base.Fitness, weights=(1.0,1.0))
elif (registered_envs[env_name]['selection']=="FIT"):
    creator.create("MyFitness", base.Fitness, weights=(1.0,))
elif (registered_envs[env_name]['selection']=="NS"):
    creator.create("MyFitness", base.Fitness, weights=(1.0,))
elif (registered_envs[env_name]['selection']=="NSLC"):
    creator.create("MyFitness", base.Fitness, weights=(1.0,1.0))
else:
    print("Variante inconnue: "+registered_envs[env_name]['selection'])

creator.create("Individual", array.array, typecode="d", fitness=creator.MyFitness, strategy=None)
creator.create("Strategy", array.array, typecode="d")

# Individual generator
def generateES(icls, scls, size, imin, imax, smin, smax):
    ind = icls(random.uniform(imin, imax) for _ in range(size))
    ind.strategy = scls(random.uniform(smin, smax) for _ in range(size))
    return ind

def checkStrategy(minstrategy):
    def decorator(func):
        def wrappper(*args, **kargs):
            children = func(*args, **kargs)
            for child in children:
                for i, s in enumerate(child.strategy):
                    if s < minstrategy:
                        child.strategy[i] = minstrategy
            return children
        return wrappper
    return decorator

IND_SIZE=registered_envs[env_name]["gen_size"]

grid={}


def launch_ea(mu=100, lambda_=200, cxpb=0.3, mutpb=0.7, ngen=100, verbose=False, resdir="res"):

    random.seed()

    # Preparation of the EA with the DEAP framework. See https://deap.readthedocs.io for more details.
    toolbox = base.Toolbox()
    toolbox.register("individual", generateES, creator.Individual, creator.Strategy, IND_SIZE, 
                     registered_envs[env_name]["min_value"], 
                     registered_envs[env_name]["max_value"], 
                     registered_envs[env_name]["min_strategy"], 
                     registered_envs[env_name]["max_strategy"])

    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("mate", tools.cxESBlend, alpha=0.1)
    toolbox.register("mutate", tools.mutESLogNormal, c=1.0, indpb=0.03)
    toolbox.register("select", tools.selNSGA2)
    
    toolbox.register("map",futures.map)
    toolbox.decorate("mate", checkStrategy(registered_envs[env_name]["min_strategy"]))
    toolbox.decorate("mutate", checkStrategy(registered_envs[env_name]["min_strategy"]))
    toolbox.register("evaluate", eval_ka, resdir=resdir, **registered_envs[env_name]["eval_params"])


    population = toolbox.population(n=mu) # should be n=mu, but it makes the results more interesting
    paretofront = tools.ParetoFront()
    
    fbd=open(resdir+"/bd.log","w")
    finfo=open(resdir+"/info.log","w")
    ffit=open(resdir+"/fit.log","w")

    nb_eval=0

    ##
    ### Initial random generation: beginning
    ##

    # Evaluate the individuals with an invalid (i.e. not yet evaluated) fitness
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses_bds = toolbox.map(toolbox.evaluate, invalid_ind)
    nb_eval+=len(invalid_ind)
    finfo.write("## Generation 0 \n")
    finfo.flush()
    ffit.write("## Generation 0 \n")
    ffit.flush()
    print("env_name : ", env_name)
    for ind, (fit, bd, log) in zip(invalid_ind, fitnesses_bds):
        #print("Fit: "+str(fit)) 
        #print("BD: "+str(bd))
        
        if (registered_envs[env_name]['selection']=="FIT+NS"):
            ind.fitness.values=(fit,0)
        elif (registered_envs[env_name]['selection']=="FIT"):
            ind.fitness.values=(fit,)
        elif (registered_envs[env_name]['selection']=="NS"):
            ind.fitness.values=(0,)
        elif (registered_envs[env_name]['selection']=="NSLC"):
            ind.fitness.values=(0,0)
        
        ind.fit = fit
        ind.log = log
        ind.bd = bd
        fbd.write(" ".join(map(str,bd))+"\n")
        fbd.flush()
        finfo.write(str(log)+"\n")
        finfo.flush()
        ffit.write(str(fit)+"\n")
        ffit.flush()
        grid_management.add_to_grid(grid,ind, fit, 
                                    dim=registered_envs[env_name]['dim_grid'], 
                                    min_v=registered_envs[env_name]['grid_min_v'], 
                                    max_v=registered_envs[env_name]['grid_max_v'])
        
    if paretofront is not None:
        paretofront.update(population)


    archive=updateNovelty(population,population,None,
                          registered_envs[env_name]['nov_k'],
                          registered_envs[env_name]['nov_add_strategy'],
                          registered_envs[env_name]['nov_lambda'])

    for ind in population:
        if (registered_envs[env_name]['selection']=="FIT+NS"):
            ind.fitness.values=(ind.fit,ind.novelty)
        elif (registered_envs[env_name]['selection']=="FIT"):
            ind.fitness.values=(ind.fit,)
        elif (registered_envs[env_name]['selection']=="NS"):
            ind.fitness.values=(ind.novelty,)

        print("Fit=%f Nov=%f "%(ind.fit, ind.novelty)+" BD="+str(ind.bd))

    ## à compléter pour calculer la valeur max de fitness. Pour que cela fonctionne de façon équivalente pour toutes les variantes
    ## Vous pourrez l'extraire du champ log, en allant chercher la valeur associée à la clé registered_envs[env_name]['watch_max']].
    print("registered_envs : ", registered_envs[env_name]['watch_max'])
    print("log : ", ind.log)

    ##
    ### Initial random generation: end
    ##


    # Begin the generational process
    for gen in range(1, ngen + 1):
        finfo.write("## Generation %d \n"%(gen))
        finfo.flush()
        ffit.write("## Generation %d \n"%(gen))
        ffit.flush()
        if (gen%10==0):
            print("+",end="", flush=True)
        else:
            print(".",end="", flush=True)

        # Vary the population
        offspring = algorithms.varOr(population, toolbox, lambda_, cxpb, mutpb)


        # Evaluate the individuals with an invalid (i.e. not yet evaluated) fitness
        invalid_ind = [ind for ind in offspring]
        fitnesses_bds = toolbox.map(toolbox.evaluate, invalid_ind)
        nb_eval+=len(invalid_ind)

        for ind, (fit, bd, log) in zip(invalid_ind, fitnesses_bds):
            #print("Fit: "+str(fit)+" BD: "+str(bd)) 
            if (registered_envs[env_name]['selection']=="FIT+NS"):
                ind.fitness.values=(fit,0)
            elif (registered_envs[env_name]['selection']=="FIT"):
                ind.fitness.values=(fit,)
            elif (registered_envs[env_name]['selection']=="NS"):
                ind.fitness.values=(0,)
            elif (registered_envs[env_name]['selection']=="NSLC"):
                ind.fitness.values=(0,0)
            ind.fit = fit
            ind.bd = bd
            ind.log=log
            fbd.write(" ".join(map(str,bd))+"\n")
            fbd.flush()
            finfo.write(str(log)+"\n")
            finfo.flush()
            ffit.write(str(fit)+"\n")
            ffit.flush()

            grid_management.add_to_grid(grid,ind, ind.fit, 
                                        dim=registered_envs[env_name]['dim_grid'], 
                                        min_v=registered_envs[env_name]['grid_min_v'], 
                                        max_v=registered_envs[env_name]['grid_max_v'])


        pq=population+offspring

        archive=updateNovelty(pq,offspring,archive,
                              registered_envs[env_name]['nov_k'],
                              registered_envs[env_name]['nov_add_strategy'],
                              registered_envs[env_name]['nov_lambda'])

        #print("Before selection: ")
        for ind in pq:
            if (registered_envs[env_name]['selection']=="FIT+NS"):
                ind.fitness.values=(ind.fit,ind.novelty)
            elif (registered_envs[env_name]['selection']=="FIT"):
                ind.fitness.values=(ind.fit,)
            elif (registered_envs[env_name]['selection']=="NS"):
                ind.fitness.values=(ind.novelty,)

            ##print("Fitness values: "+str(ind.fitness.values)+" Fit=%f Nov=%f"%(ind.fit, ind.novelty))
        

        # Select the next generation population
        population[:] = toolbox.select(pq, mu)

        ##print("After selection: ")
        ##for ind in population:
        ##    print("Fitness values: "+str(ind.fitness.values)+" Fit=%f Nov=%f"%(ind.fit, ind.novelty))

        # Update the hall of fame with the generated individuals
        if paretofront is not None:
            paretofront.update(population)

        ## à compléter pour mettre à jour la valeur max si besoin, afficher un message et garder une trace de l'individu si mise à jour il y a. 

    fbd.close()
    finfo.close()
    ffit.close()

    
    grid_management.stat_grid(grid, resdir, nb_eval, dim=registered_envs[env_name]['dim_grid'])
    grid_management.dump_grid(grid, resdir, dim=registered_envs[env_name]['dim_grid'])

    return population, None, paretofront, grid

#env = gym.make(registered_envs[env_name]['gym_name'], **registered_envs[env_name]['env_params'])


if (__name__ == "__main__"):

    resdir="res_"+env_name+"_"+datetime.datetime.now().strftime("%Y_%m_%d_%H:%M:%S")
    os.mkdir(resdir)
    ngen=registered_envs[env_name]['nb_gen']
    lambda_=registered_envs[env_name]['lambda']
    mu=registered_envs[env_name]['mu']

    with open(resdir+"/run_params.log", "w") as rf:
        rf.write("env_name: "+env_name)
        for k in registered_envs[env_name].keys():
            rf.write(k+": "+str(registered_envs[env_name][k])+"\n")

    pop, logbook, paretofront, grid = launch_ea(mu=mu, lambda_=lambda_, ngen=ngen, resdir=resdir)


    cdir="completed_runs"
    try:
        os.mkdir(cdir)
    except FileExistsError:
        pass
    os.rename(resdir,cdir+"/"+resdir) 

    #env.close()

    print("Results saved in "+cdir+"/"+resdir)

