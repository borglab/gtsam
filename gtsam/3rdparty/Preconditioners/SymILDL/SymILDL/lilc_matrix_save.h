//-*-mode:c++-*-
#ifndef _LILC_MATRIX_SAVE_H_
#define _LILC_MATRIX_SAVE_H_

inline void put_header(std::string& header, bool sym = false)
{
	header= "%%MatrixMarket matrix coordinate real ";
	if (sym)
		header += "symmetric"; //maybe change later to have symmetric/complex/blah as options
	else
		header += "general";
}

template <class el_type>
bool lilc_matrix<el_type> :: save(std::string filename, bool sym)
{
	std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
	if(!out)
	return false;

	out.flags(std::ios_base::scientific);
	out.precision(16);
	std::string header; 
	put_header(header, sym); 

	out << header << std::endl; 
	out << n_rows() << " " << n_cols() << " " << nnz() << "\n";

	for(int i = 0; i < n_cols(); i++) {
		for(unsigned int j = 0; j < m_idx[i].size(); j++) {
			out << m_idx[i][j]+1 << " " << i+1 << " " << m_x[i][j] << "\n";
		}
	}
	
	out.close();
	return true;
}

#endif
