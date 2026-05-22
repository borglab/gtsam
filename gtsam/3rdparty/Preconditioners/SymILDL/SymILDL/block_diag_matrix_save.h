//-*-mode:c++-*-
#ifndef _BLOCK_DIAG_MATRIX_SAVE_H_
#define _BLOCK_DIAG_MATRIX_SAVE_H_

template <class el_type>
bool block_diag_matrix<el_type> :: save(std::string filename) const
{
	std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
	if(!out)
	return false;

	out.flags(std::ios_base::scientific);
	out.precision(16);
	std::string header;
	
	header= "%%MatrixMarket matrix coordinate ";
	header += "real symmetric"; //maybe change later to have general/complex/blah as options

	out << header << std::endl; 
	out << n_rows() << " " << n_cols() << " " << nnz() << "\n";

	for(int i = 0; i < n_cols(); i++) {
		out << i+1 << " " << i+1 << " " << main_diag[i] << "\n";
		if (block_size(i) == 2) {
			out << i+2 << " " << i+1 << " " << off_diag.find(i)->second << "\n";
			out << i+2 << " " << i+2 << " " << main_diag[i+1] << "\n";
			i++;
		}
	}
	
	out.close();
	return true;
}

#endif
