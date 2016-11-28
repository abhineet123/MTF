#include "mtf/SM/RegNetParams.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

RegNetParams::RegNetParams(
	int _max_iters, int _n_samples, double _epsilon,
	const vectorvd &_ssm_sigma, vectorvd _ssm_mean,
	vectord _pix_sigma, bool _additive_update, 
	int _show_samples, int _add_points, int _remove_points,
	bool _load_index, bool _save_index, string _saved_index_dir,
	bool _debug_mode, int _n_epochs, int _bs, bool _preproc, 
    char *_solver_f, char * _train_f, char * _mean_f,
    bool _debug, bool _pretrained ) :
	max_iters(_max_iters),
	n_samples(_n_samples),
	epsilon(_epsilon),
	ssm_sigma(_ssm_sigma),
	ssm_mean(_ssm_mean),
	pix_sigma(_pix_sigma),
	additive_update(_additive_update),
	show_samples(_show_samples),
	add_points(_add_points),
	remove_points(_remove_points),
	load_index(_load_index),
	save_index(_save_index),
	saved_index_dir(_saved_index_dir),
	debug_mode(_debug_mode), nepochs(_n_epochs), bs(_bs),
    enable_preproc(_preproc), solver_file(_solver_f), train_file(_train_f),
    mean_file(_mean_f), debug(_debug), load_pretrained(_pretrained){}

RegNetParams::RegNetParams(const RegNetParams *params) :
max_iters(RG_MAX_ITERS),
n_samples(RG_N_SAMPLES),
epsilon(RG_EPSILON),
additive_update(RG_ADDITIVE_UPDATE),
show_samples(RG_SHOW_SAMPLES),
add_points(RG_ADD_POINTS),
remove_points(RG_REMOVE_POINTS),
load_index(RG_LOAD_INDEX),
save_index(RG_SAVE_INDEX),
saved_index_dir(RG_INDEX_FILE_TEMPLATE),
debug_mode(RG_DEBUG_MODE), nepochs(NEPOCHS), bs(BS),
enable_preproc(PREPROC), solver_file(SOLVERF), train_file(TRAINF),
mean_file(MEANF), debug(DEBUG), load_pretrained(PRETRAINED)
{
	if(params){
		max_iters = params->max_iters;
		n_samples = params->n_samples;
		epsilon = params->epsilon;

		ssm_sigma = params->ssm_sigma;
		ssm_mean = params->ssm_mean;
		pix_sigma = params->pix_sigma;

		additive_update = params->additive_update;
		show_samples = params->show_samples;

		add_points = params->add_points;
		remove_points = params->remove_points;

		load_index = params->load_index;
		save_index = params->save_index;
		saved_index_dir = params->saved_index_dir;

		debug_mode = params->debug_mode;

        nepochs= params->nepochs;
        bs= params->bs;

        enable_preproc= params->enable_preproc;
        solver_file= params->solver_file;
        train_file= params->train_file;
        mean_file= params->mean_file;
        debug= params->debug;
        load_pretrained= params->load_pretrained;
	}
}

_MTF_END_NAMESPACE
