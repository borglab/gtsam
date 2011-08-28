/*
	Multi-precision real number class. C++ interface fo MPFR library.
	Project homepage: http://www.holoborodko.com/pavel/
	Contact e-mail:   pavel@holoborodko.com

	Copyright (c) 2008-2010 Pavel Holoborodko

	Core Developers: 
	Pavel Holoborodko, Dmitriy Gubanov, Konstantin Holoborodko. 

	Contributors:
	Brian Gladman, Helmut Jarausch, Fokko Beekhof, Ulrich Mutze, 
	Heinz van Saanen, Pere Constans, Peter van Hoof.

	****************************************************************************
	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

	****************************************************************************
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:
	
	1. Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.
	
	2. Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.
	
	3. Redistributions of any form whatsoever must retain the following
	acknowledgment:
	"
         This product includes software developed by Pavel Holoborodko
         Web: http://www.holoborodko.com/pavel/
         e-mail: pavel@holoborodko.com
	"

	4. This software cannot be, by any means, used for any commercial 
	purpose without the prior permission of the copyright holder.
	
	Any of the above conditions can be waived if you get permission from 
	the copyright holder. 

	THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
	OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
	OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
	SUCH DAMAGE.
*/
#include <cstring>
#include <cstdlib>
#include "mpreal.h"
#include "dlmalloc.h"

using std::ws;
using std::cerr;
using std::endl;
using std::string;
using std::ostream;
using std::istream;

namespace mpfr{

mp_rnd_t   mpreal::default_rnd  = mpfr_get_default_rounding_mode();	
mp_prec_t  mpreal::default_prec = mpfr_get_default_prec();	
int		   mpreal::default_base = 10;
int        mpreal::double_bits = -1;
bool       mpreal::is_custom_malloc = false;

// Default constructor: creates mp number and initializes it to 0.
mpreal::mpreal() 
{ 
	set_custom_malloc();
	mpfr_init2(mp,default_prec); 
	mpfr_set_ui(mp,0,default_rnd);
}

mpreal::mpreal(const mpreal& u) 
{
	set_custom_malloc();
	mpfr_init2(mp,mpfr_get_prec(u.mp));
	mpfr_set(mp,u.mp,default_rnd);
}

mpreal::mpreal(const mpfr_t u)
{
	set_custom_malloc();
	mpfr_init2(mp,mpfr_get_prec(u));
	mpfr_set(mp,u,default_rnd);
}

mpreal::mpreal(const mpf_t u)
{
	set_custom_malloc();
	mpfr_init2(mp,mpf_get_prec(u));
	mpfr_set_f(mp,u,default_rnd);
}

mpreal::mpreal(const mpz_t u, mp_prec_t prec, mp_rnd_t mode)
{
	set_custom_malloc();
	mpfr_init2(mp,prec);
	mpfr_set_z(mp,u,mode);
}

mpreal::mpreal(const mpq_t u, mp_prec_t prec, mp_rnd_t mode)
{
	set_custom_malloc();
	mpfr_init2(mp,prec);
	mpfr_set_q(mp,u,mode);
}

mpreal::mpreal(const double u, mp_prec_t prec, mp_rnd_t mode)
{
	set_custom_malloc();
    if(double_bits == -1 || fits_in_bits(u, double_bits))
    {
    	mpfr_init2(mp,prec);
	    mpfr_set_d(mp,u,mode);
    }
    else
        throw conversion_overflow();
}

mpreal::mpreal(const long double u, mp_prec_t prec, mp_rnd_t mode)
{ 
	set_custom_malloc();
    mpfr_init2(mp,prec);
	mpfr_set_ld(mp,u,mode);
}

mpreal::mpreal(const unsigned long int u, mp_prec_t prec, mp_rnd_t mode)
{ 
	set_custom_malloc();
	mpfr_init2(mp,prec);
	mpfr_set_ui(mp,u,mode);
}

mpreal::mpreal(const unsigned int u, mp_prec_t prec, mp_rnd_t mode)
{ 
	set_custom_malloc();
	mpfr_init2(mp,prec);
	mpfr_set_ui(mp,u,mode);
}

mpreal::mpreal(const long int u, mp_prec_t prec, mp_rnd_t mode)
{ 
	set_custom_malloc();
	mpfr_init2(mp,prec);
	mpfr_set_si(mp,u,mode);
}

mpreal::mpreal(const int u, mp_prec_t prec, mp_rnd_t mode)
{ 
	set_custom_malloc();
	mpfr_init2(mp,prec);
	mpfr_set_si(mp,u,mode);
}

mpreal::mpreal(const char* s, mp_prec_t prec, int base, mp_rnd_t mode)
{
	set_custom_malloc();
	mpfr_init2(mp,prec);
	mpfr_set_str(mp, s, base, mode); 
}

mpreal::mpreal(const std::string& s, mp_prec_t prec, int base, mp_rnd_t mode)
{
	set_custom_malloc();
	mpfr_init2(mp,prec);
	mpfr_set_str(mp, s.c_str(), base, mode); 
}

mpreal::~mpreal() 
{ 
	mpfr_clear(mp);
}                           

// Operators - Assignment
mpreal& mpreal::operator=(const char* s)
{
	mpfr_t t;
	
	set_custom_malloc();

	if(0==mpfr_init_set_str(t,s,default_base,default_rnd))
	{
		// We will rewrite mp anyway, so use flash it and resize
		mpfr_set_prec(mp,mpfr_get_prec(t)); //<- added 01.04.2011
		mpfr_set(mp,t,mpreal::default_rnd);
		mpfr_clear(t);
	}else{
		mpfr_clear(t);
		// cerr<<"fail to convert string"<<endl;
	}

	return *this;
}

const mpreal fma (const mpreal& v1, const mpreal& v2, const mpreal& v3, mp_rnd_t rnd_mode)
{
	mpreal a;
	mp_prec_t p1, p2, p3;

	p1 = v1.get_prec(); 
	p2 = v2.get_prec(); 
	p3 = v3.get_prec(); 

	a.set_prec(p3>p2?(p3>p1?p3:p1):(p2>p1?p2:p1));

	mpfr_fma(a.mp,v1.mp,v2.mp,v3.mp,rnd_mode);
	return a;
}

const mpreal fms (const mpreal& v1, const mpreal& v2, const mpreal& v3, mp_rnd_t rnd_mode)
{
	mpreal a;
	mp_prec_t p1, p2, p3;

	p1 = v1.get_prec(); 
	p2 = v2.get_prec(); 
	p3 = v3.get_prec(); 

	a.set_prec(p3>p2?(p3>p1?p3:p1):(p2>p1?p2:p1));

	mpfr_fms(a.mp,v1.mp,v2.mp,v3.mp,rnd_mode);
	return a;
}

const mpreal agm (const mpreal& v1, const mpreal& v2, mp_rnd_t rnd_mode)
{
	mpreal a;
	mp_prec_t p1, p2;

	p1 = v1.get_prec(); 
	p2 = v2.get_prec(); 

	a.set_prec(p1>p2?p1:p2);

	mpfr_agm(a.mp, v1.mp, v2.mp, rnd_mode);

	return a;
}

const mpreal hypot (const mpreal& x, const mpreal& y, mp_rnd_t rnd_mode)
{
	mpreal a;
	mp_prec_t yp, xp;

	yp = y.get_prec(); 
	xp = x.get_prec(); 

	a.set_prec(yp>xp?yp:xp);

	mpfr_hypot(a.mp, x.mp, y.mp, rnd_mode);

	return a;
}

const mpreal sum (const mpreal tab[], unsigned long int n, mp_rnd_t rnd_mode)
{
	mpreal x;
	mpfr_ptr* t;
	unsigned long int i;

	t = new mpfr_ptr[n];
	for (i=0;i<n;i++) t[i] = (mpfr_ptr)tab[i].mp;
	mpfr_sum(x.mp,t,n,rnd_mode);
	delete[] t;
	return x;
}

const mpreal remainder (const mpreal& x, const mpreal& y, mp_rnd_t rnd_mode)
{	
	mpreal a;
	mp_prec_t yp, xp;

	yp = y.get_prec(); 
	xp = x.get_prec(); 

	a.set_prec(yp>xp?yp:xp);

	mpfr_remainder(a.mp, x.mp, y.mp, rnd_mode);

	return a;
}

const mpreal remquo (long* q, const mpreal& x, const mpreal& y, mp_rnd_t rnd_mode)
{
	mpreal a;
	mp_prec_t yp, xp;

	yp = y.get_prec(); 
	xp = x.get_prec(); 

	a.set_prec(yp>xp?yp:xp);

	mpfr_remquo(a.mp,q, x.mp, y.mp, rnd_mode);

	return a;
}

template <class T>
std::string to_string(T t, std::ios_base & (*f)(std::ios_base&))
{
	std::ostringstream oss;
	oss << f << t;
	return oss.str();
}

mpreal::operator std::string() const
{
	return to_string();
}

std::string mpreal::to_string(size_t n, int b, mp_rnd_t mode) const
{
	char *s, *ns = NULL;	
	size_t slen, nslen;
	mp_exp_t exp;
	string out;
	
	set_custom_malloc();
	
	if(mpfr_inf_p(mp))
	{ 
		if(mpfr_sgn(mp)>0) return "+@Inf@";
		else			   return "-@Inf@";
	}

	if(mpfr_zero_p(mp)) return "0";
	if(mpfr_nan_p(mp))  return "@NaN@";
		
	s  = mpfr_get_str(NULL,&exp,b,0,mp,mode);
	ns = mpfr_get_str(NULL,&exp,b,n,mp,mode);

	if(s!=NULL && ns!=NULL)
	{
		slen  = strlen(s);
		nslen = strlen(ns);
		if(nslen<=slen) 
		{
			mpfr_free_str(s);
			s = ns;
			slen = nslen;
		}
		else {
			mpfr_free_str(ns);
		}

		// Make human eye-friendly formatting if possible
		if (exp>0 && static_cast<size_t>(exp)<slen)
		{
			if(s[0]=='-')
			{
				// Remove zeros starting from right end
				char* ptr = s+slen-1;
				while (*ptr=='0' && ptr>s+exp) ptr--; 

				if(ptr==s+exp) out = string(s,exp+1);
				else		   out = string(s,exp+1)+'.'+string(s+exp+1,ptr-(s+exp+1)+1);

				//out = string(s,exp+1)+'.'+string(s+exp+1);
			}
			else
			{
				// Remove zeros starting from right end
				char* ptr = s+slen-1;
				while (*ptr=='0' && ptr>s+exp-1) ptr--; 

				if(ptr==s+exp-1) out = string(s,exp);
				else		     out = string(s,exp)+'.'+string(s+exp,ptr-(s+exp)+1);

				//out = string(s,exp)+'.'+string(s+exp);
			}

		}else{ // exp<0 || exp>slen
			if(s[0]=='-')
			{
				// Remove zeros starting from right end
				char* ptr = s+slen-1;
				while (*ptr=='0' && ptr>s+1) ptr--; 

				if(ptr==s+1) out = string(s,2);
				else		 out = string(s,2)+'.'+string(s+2,ptr-(s+2)+1);

				//out = string(s,2)+'.'+string(s+2);
			}
			else
			{
				// Remove zeros starting from right end
				char* ptr = s+slen-1;
				while (*ptr=='0' && ptr>s) ptr--; 

				if(ptr==s) out = string(s,1);
				else	   out = string(s,1)+'.'+string(s+1,ptr-(s+1)+1);

				//out = string(s,1)+'.'+string(s+1);
			}

			// Make final string
			if(--exp)
			{
				if(exp>0) out += "e+"+mpfr::to_string<mp_exp_t>(exp,std::dec);
				else 	  out += "e"+mpfr::to_string<mp_exp_t>(exp,std::dec);
			}
		}

		mpfr_free_str(s);
		return out;
	}else{
		return "conversion error!";
	}
}

//////////////////////////////////////////////////////////////////////////
// I/O
ostream& operator<<(ostream& os, const mpreal& v)
{
	return os<<v.to_string(static_cast<size_t>(os.precision()));
}

istream& operator>>(istream &is, mpreal& v)
{
	char c;	
	string s = "";
	mpfr_t t;
	
	mpreal::set_custom_malloc();
	
	if(is.good())
	{
		is>>ws;
		while ((c = is.get())!=EOF)
		{
			if(c ==' ' || c == '\t' || c == '\n' || c == '\r')
			{
				is.putback(c);
				break;
			}
			s += c;
		}

		if(s.size() != 0)
		{
			// Protect current value from alternation in case of input error
			// so some error handling(roll back) procedure can be used 			

			if(0==mpfr_init_set_str(t,s.c_str(),mpreal::default_base,mpreal::default_rnd))
			{
				mpfr_set(v.mp,t,mpreal::default_rnd);				
				mpfr_clear(t);

			}else{
				mpfr_clear(t);
				cerr<<"error reading from istream"<<endl;
				// throw an exception
			}
		}
	}
	return is;
}

// Optimized dynamic memory allocation/(re-)deallocation.
void * mpreal::mpreal_allocate(size_t alloc_size)
{
	return(dlmalloc(alloc_size));
}

void * mpreal::mpreal_reallocate(void *ptr, size_t /*old_size*/, size_t new_size)
{
	return(dlrealloc(ptr,new_size));
}

void mpreal::mpreal_free(void *ptr, size_t /*size*/)
{
	dlfree(ptr);
}

inline void mpreal::set_custom_malloc(void)
{
	if(!is_custom_malloc)
	{
		mp_set_memory_functions(mpreal_allocate,mpreal_reallocate,mpreal_free);
		is_custom_malloc = true;
	}
}
}

