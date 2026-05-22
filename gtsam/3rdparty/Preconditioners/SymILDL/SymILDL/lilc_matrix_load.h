//-*-mode:c++-*-
#ifndef _LILC_MATRIX_LOAD_H_
#define _LILC_MATRIX_LOAD_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

template <class el_type>
inline bool readline (std::stringstream& line, int& n_rows, int& n_cols, int& i, int& j, el_type& value) {
	line >> i >> j >> value;
	i--;
	j--;
	if(i>=0 && j>=0 && i<n_rows && j< n_cols) {
		return true; 
	}
	else
	return false;
}

template <class el_type>
bool lilc_matrix<el_type> :: load (std::string filename)
{
	std::ifstream input(filename.c_str(), std::ios::in);
	//input.sync_with_stdio(0);

	if(!input) return false;
	
	const int maxBuffersize = 2048;
	char buffer[maxBuffersize];

	bool readsizes = false;

	int n_rows(-1), n_cols(-1), n_nzs(-1), i(-1), j(-1);
	int count = 0;
	el_type value; 

	bool full_detected = false;
	while(input.getline(buffer, maxBuffersize))
	{
		// skip comments   
		//NOTE An appropriate test should be done on the header to get the symmetry
		if(buffer[0]=='%')
		continue;
		
		std::stringstream line(buffer);
		//line.sync_with_stdio(0);
		if(!readsizes)
		{
			line >> n_rows >> n_cols >> n_nzs;
			if(n_rows > 0 && n_cols > 0 && n_nzs > 0) 
			{
				readsizes = true;
				
				resize(n_rows, n_cols);
				std::fill(row_first.begin(), row_first.end(), 0); //a bit of optimization could be used here since resize sets all elem in first to 1
				std::fill(col_first.begin(), col_first.end(), 0); //a bit of optimization could be used here since resize sets all elem in first to 1
			}
		}
		else
		{ 
			i = -1;
			j = -1;
			if( readline(line, n_rows, n_cols, i, j, value) ) 
			{
				if (j > i) {
					full_detected = true;
					continue;
				}
				m_idx[j].push_back(i);
				m_x[j].push_back(value);
				++count;
				assert(i >= j);
				if (i != j) list[i].push_back(j);
				
			}
			else 
			std::cerr << "Invalid read: " << i << "," << j << "\n";		
		}
		
	}
	
	if (!full_detected && count != n_nzs) std::cout << "Expected " << n_nzs << " elems but read " << count << "." << std::endl;
	
	if (full_detected) {
		std::cout << "Full matrix detected, assuming matrix is symmetric and loading lower-half of the matrix only." << std::endl;
	}
	nnz_count = count;
	std::cout << "Load succeeded. " << "File " << filename << " was loaded." << std::endl;
	input.close();
	return true;
}

template<class el_type>
bool lilc_matrix<el_type> :: load (const std::vector<int>& ptr, const std::vector<int>& row, const std::vector<el_type>& val) {
	if (ptr.size() == 0 || ptr.back() != row.size() || val.size() != ptr.back()) {
		std::cout << "Error in CSC format detected. Matrix failed to load." << std::endl;
		return false;
	}
	return load(ptr.data(), row.data(), val.data(), ptr.size()-1);
}

template<class el_type>
bool lilc_matrix<el_type> :: load (const int* ptr, const int* row, const el_type* val, int dim) {
	bool full_detected = false;
	int n_rows = dim, n_cols = dim;
	
	resize(n_rows, n_cols);
	std::fill(row_first.begin(), row_first.end(), 0); //a bit of optimization could be used here since resize sets all elem in first to 1
	std::fill(col_first.begin(), col_first.end(), 0); //a bit of optimization could be used here since resize sets all elem in first to 1
	
	int count = 0;
	for (int i = 0; i < dim; i++) {
		for (int j = ptr[i]; j < ptr[i+1]; j++) {
			if (i > row[j]) {
				full_detected = true;
				continue;
			}
			m_idx[i].push_back(row[j]);
			m_x[i].push_back(val[j]);
			if (i != row[j]) list[row[j]].push_back(i);
			++count;
		}
	}

	if (full_detected) {
		std::cout << "Full matrix detected, assuming matrix is symmetric and loading lower-half of the matrix only." << std::endl;
	}
	
	nnz_count = count;
	return true;
}

#endif
