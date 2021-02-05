/*
 * Matrix.c
 *
 * Created: 27-Apr-18 19:29:15
 *  Author: Willem van der Kooij
 */ 

#include "sam.h"
#include <component/matrix.h>
#include "Matrix.h"


void Setup_Matrix(void)
{
	/* Setup MATRIX[1] (processor system bus) slave peripheral bridge (MATRIX[3]) */
	MATRIX -> MATRIX_WPMR &=~(MATRIX_WPMR_WPEN);
	MATRIX -> MATRIX_WPMR = (MATRIX_WPMR_WPKEY_PASSWD);
	
	//set I2S clock
	MATRIX -> CCFG_I2SCLKSEL = CCFG_I2SCLKSEL_CLKSEL0; //run I2SC of PCK4
		
	/*
	MATRIX -> MATRIX_SCFG[3]	= MATRIX_SCFG_DEFMSTR_TYPE_FIXED
								| MATRIX_SCFG_FIXED_DEFMSTR(1);

	MATRIX -> MATRIX_MCFG[1]	= MATRIX_MCFG_ULBT_UNLIMITED; //no predicted end of burst is generated

	MATRIX -> CCFG_DYNCKG		= CCFG_DYNCKG_MATCKG
								| CCFG_DYNCKG_BRIDCKG;
	*/
}