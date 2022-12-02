import random
import functools
import math

def get_grid_coord(bd, dim=[100, 100], min_v=[0,0], max_v=[1, 1]):
    # Returns the coordinates of the grid as a tuple (so that it is hashable)
    return (0,0) # TO MODIFY

def add_to_grid(grid,ind, local_quality, dim=[100, 100], min_v=[0,0], max_v=[1, 1]):
    # determining the grid cell coordinates to add to

    # TO MODIFY

    pass

def stat_grid(grid, resdir, nb_eval, dim=[100, 100]):
    if(len(grid.values())==0):
        print("Empty grid: no stats...")
        return

    nb_filled=0
    max_v=None
    for i in range(dim[0]):
        for j in range(dim[1]):
            if ((i,j) in grid.keys()):
                nb_filled+=1
                
    nbcells=functools.reduce(lambda x, y: x*y, dim)
    c_values=[ind.fit for ind in list(grid.values())]
    max_v=max(c_values)
    total_quality=sum(c_values)
    #print("Number of evaluations: %d"%(nb_eval))
    print("Coverage: %.2f %% (%d cells out of %d)"%(float(nb_filled)/float(nbcells)*100., nb_filled, nbcells)+" Max score: %.2f"%(max(c_values))+" Min score: %.2f"%(min(c_values))+" Total quality: %.2f"%(total_quality))
    stat_grid={
        'nb_eval': nb_eval,
        'nb_cells': nbcells,
        'nb_filled': nb_filled,
        'max_score': max(c_values),
        'min_score': min(c_values),
        'coverage': float(nb_filled)/float(nbcells),
        }
    with open(resdir+"/stat_grid.log","a") as sf:
        sf.write(str(stat_grid)+"\n")
        


def dump_grid(grid, resdir, dim=[100, 100]):
    if(len(grid.values())==0):
        print("Empty grid: no dump...")
        return
    with open(resdir+"/map.dat","w") as mf:
        for i in range(dim[0]):
            for j in range(dim[1]):
                if ((i,j) in grid.keys()):
                    mf.write("%.2f "%(grid[(i,j)].fit))
                else:
                    mf.write("=== ")
            mf.write("\n")
    with open(resdir+"/map_bd.dat","w") as mf:
        for p in grid.keys():
            ind=grid[p]
            for i in range(len(ind.bd)):
                mf.write("%f "%(ind.bd[i]))
            mf.write("\n")
