# -*- coding: utf-8 -*-
"""
Created on Thu Jul 21 13:47:59 2022

@author: RLaje, ASilva
"""

import numpy as np
import pandas as pd
import glob
import os


#%% Define Python user-defined exceptions.
class Error(Exception):
	"""Base class for other exceptions"""
	pass


#%% Condition_Dictionary
# Function to create the condition dictionary. Return a dataframe.
# perturb_type_dictionary --> dict. perturb_size_dictionary --> dict.
def Condition_Dictionary(perturb_type_dictionary, perturb_size_dictionary):

	condition_list = []
	condition_number_list = []
	perturb_type_list = []
	perturb_size_list = []
	
	index = 0
	for perturb_type in perturb_type_dictionary.keys():
		for perturb_size in perturb_size_dictionary.keys():
			condition_list.append(perturb_type + perturb_size)	
			condition_number_list.append(index)
			perturb_type_list.append(int(perturb_type_dictionary[perturb_type]))
			perturb_size_list.append(int(perturb_size_dictionary[perturb_size]))
			index = index + 1
	
	condition_dictionary_df = pd.DataFrame()
	condition_dictionary_df = condition_dictionary_df.assign(Name = condition_list, Condition = condition_number_list, 
								Perturb_type = perturb_type_list, Perturb_size = perturb_size_list)
	return condition_dictionary_df


#%% compute_asyn
# Compute asynchronies from stimulus and response time occurrences. Return a dataframe.
def Compute_Asyn(stim_times,resp_times):
	if len(resp_times)==0:
		asyn = []
		assigned_stim = []
	else:
		ISI = np.median(np.diff(stim_times))
		asyn_max = round(ISI/2)
		n_stims = len(stim_times)
		n_resps = len(resp_times)
		minima_R = np.zeros((n_stims,n_resps),dtype=int)
		minima_S = np.zeros((n_stims,n_resps),dtype=int)
		assigned_stim = -np.ones(n_resps,dtype=int)
		stimuli_flag = np.zeros(n_stims,dtype=int)
		asyn = np.full(n_resps,np.nan)
	
		# Find matching S-R pairs
	
		# pairwise differences between every response and every stimulus
		# (dimensions = number of stimuli x number of responses)
		differences = -np.subtract.outer(stim_times,resp_times).astype(float)	# type float so it can be NaN
		differences[abs(differences)>=asyn_max] = np.nan # remove differences larger than threshold
	
		# for every response, find the closest stimulus (nontrivial if more responses than stimuli)
		# IM SURE THIS LOOP CAN BE VECTORIZED
		for resp in range(n_resps):
			aux = differences[:,resp]
			# prevent "no non-missing arguments to min; returning Inf" warning
			#if np.any(np.isnan(aux)): # find at least one non-missing value
			min_abs = np.nanmin(abs(aux))
			min_idx = np.where(abs(aux)==min_abs)
			minima_R[min_idx,resp] = 1
	
	
		# remove multiple responses closest to a single stimulus (row-wise consecutive 1's)
		# i.e. make no attempt at deciding
		minima_shift_R = minima_R + np.roll(minima_R,(0,1),(0,1)) + np.roll(minima_R,(0,-1),(0,1))
		minima_R[np.where(minima_shift_R>=2)] = 0
	
		# for every stimulus, find the closest response (nontrivial if more stimuli than responses)
		# IM SURE THIS LOOP CAN BE VECTORIZED
		for stim in range(n_stims):
			aux = differences[stim,:]
			# prevent "no non-missing arguments to min; returning Inf" warning
			#if np.any(np.isnan(aux)): # find at least one non-missing value
			if all(np.isnan(aux)):
				min_abs = np.nan
			else:
				min_abs = np.nanmin(abs(aux))
			min_idx = np.where(abs(aux)==min_abs)
			minima_S[stim,min_idx] = 1
	
		# remove multiple stimuli closest to a single response (col-wise consecutive 1's)
		# i.e. make no attempt at deciding
		minima_shift_S = minima_S + np.roll(minima_S,(1,0),(0,1)) + np.roll(minima_S,(-1,0),(0,1))
		minima_S[np.where(minima_shift_S>=2)] = 0
	
		# matching pairs are represented by intersections (i.e. common 1's)
		minima_intersect = minima_R*minima_S
	
		# save asynchronies: get row and column for every matched pair
		SR_idxs = np.where(minima_intersect==1)
		S_idx = SR_idxs[0]
		R_idx = SR_idxs[1]
	
		# keep track of which stimulus was assigned to each response (-1 if not assigned)
		assigned_stim[R_idx] = S_idx
		# keep track of assigned stimuli (0 if not assigned)
		stimuli_flag[S_idx] = np.ones(len(S_idx))
	
		# save asynchrony (NaN means that response was not assigned)
		if (S_idx.size != 0 and R_idx.size != 0):
			lin_ind = np.ravel_multi_index((S_idx,R_idx),differences.shape)
			asyn[R_idx] = differences.ravel()[lin_ind]
	
	# Output
	output = pd.DataFrame({'asyn':asyn,'assigned_stim':assigned_stim})
	return output


#%% Error_Handling_For_Invalid_Trials. //AGREGAR COMENTARIO.
# Function to errors handling. Return a tuple.
# asyn_df --> dataframe from function Compute_Asyn. resp_times --> list with the response times. max_stim_for_first_resp --> maximum number of stimulus for first response. 
def Error_Handling(asyn_df, resp_times):
	
	max_stim_for_first_resp = 5	# Maximum number of stimulus for first response
	
	# Determine number of stimulus and responses registered.
	N_resp = len(resp_times)
	assigned_stim = asyn_df['assigned_stim'].values
	assigned_stim_NFR_filtered = []
	
	try: 
		if N_resp > 0: # If there were any response.

			# Find first stimulus with a decent response.
			first_assigned_stim = next(x for x in assigned_stim if x>=0)
			# If the first assigned stimulus doesn't much with any of the first stimulus, then re-do the trial.
			if first_assigned_stim >= max_stim_for_first_resp:			
				error_label = 'Tipo NFR'
				error_type = 'NoFirstResp'
				raise Error 

			# Find non assigned responses.
			if any(assigned_stim==-1):
				error_label = 'Tipo NAR'
				error_type = 'NonAssignedResp'
				raise Error

			# Find non assigned stimuli
			for i in range (len(assigned_stim)):
				if (assigned_stim[i] >= max_stim_for_first_resp):
					assigned_stim_NFR_filtered.append(assigned_stim[i])
			if (any(np.diff(assigned_stim_NFR_filtered)!=1) or assigned_stim_NFR_filtered[0] != max_stim_for_first_resp):
				error_label = 'Tipo SS'
				error_type = 'SkipStim'
				raise Error

			# If the code got here, then the trial is valid!
			valid_trial = 1
			error_label = ''
			error_type = 'NoError'                    

		else: # If there were no responses.
			# Trial is not valid! then:
			error_label = 'Tipo NR'
			error_type = 'NoResp'  
			raise Error

	except (Error):
		# Trial is not valid! then:
		valid_trial = 0
		
	return error_label, error_type, valid_trial


#%% Unfinished_Files
# Procedure to change names in unfinished files.
# subject_number --> int (ej: 1). n_block --> int (ej: 0). n_blocks --> int (ej: 2).
def Unfinished_Files(path, s_number, n_block, n_blocks):

	file_names_list = []
	original_path = os.getcwd()
	os.chdir(path)
	for block in range(n_block,n_blocks):
		file_names = glob.glob('S'+s_number+"*-block"+str(block)+"*")
		file_names_list = file_names_list + file_names
		
	for file in os.listdir():
		for searched_file in file_names_list:
			if (searched_file == file):
				src = file
				dst = 'unfinished' + file
				os.rename(src,dst)
	os.chdir(original_path)

	return

