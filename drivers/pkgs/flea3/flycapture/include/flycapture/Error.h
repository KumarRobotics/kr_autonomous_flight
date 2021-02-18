//=============================================================================
// Copyright © 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

//=============================================================================
// $Id: Error.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_ERROR_H
#define PGR_FC2_ERROR_H

#include "FlyCapture2Platform.h"
#include "FlyCapture2Defs.h"
#include <stdio.h>

namespace FlyCapture2
{
	struct ErrorImpl;

	/**
	 * The Error object represents an error that is returned from the library.
	 * Overloaded operators allow comparisons against other Error objects or
	 * the ErrorType enumeration.
	 */
	class FLYCAPTURE2_API Error
	{
		public:

			/**
			 * Default constructor.
			 */
			Error();

			/**
			 * Copy constructor.
			 */
			Error( const Error& error );

			/**
			 * Default destructor.
			 */
			virtual ~Error();

			/**
			 * Assignment operator.
			 */
			virtual Error& operator=( const Error& error );

			/**
			 * Equality operator.
			 */
			virtual bool operator==( const Error& error ) const;

			/**
			 * Equality operator. This overloaded operator compares the
			 * ErrorType of the Error against the specified ErrorType.
			 */
			virtual bool operator==( const ErrorType& errorType ) const;

			/**
			 * Inequality operator.
			 */
			virtual bool operator!=( const Error& error ) const;

			/**
			 * Inequality operator. This overloaded operator compares the
			 * ErrorType of the Error against the specified ErrorType.
			 */
			virtual bool operator!=( const ErrorType& errorType ) const;

			/**
			 * Retrieve the ErrorType of the error.
			 *
			 * @return The ErrorType of the error.
			 */
			virtual ErrorType GetType() const;

			/**
			 * Retrieve the top level description of the error that occurred.
			 *
			 * @return A string with the error description.
			 */
			virtual const char* GetDescription() const;

			/**
			 * Retrieve the line number where the error originated.
			 *
			 * @return The line number.
			 */
			virtual unsigned int GetLine() const;

			/**
			 * Retrieve the source filename where the error originated.
			 *
			 * @return A string with the file name.
			 */
			virtual const char* GetFilename() const;

			/**
			 * Get the error which caused this error.
			 *
			 * @return An error object representing the cause of this error.
			 */
			virtual Error GetCause() const;

			/**
			 * Retrieve the build date of the file where the error originated.
			 *
			 * @return A string with the build date and time.
			 */
			virtual const char* GetBuildDate() const;

			/**
			 * Retrieve the support information.
			 * It is not implemented in this release.
			 *
			 * @return A string containing support information.
			 */
			virtual const char* CollectSupportInformation() const;

			/**
			 * Print a formatted log trace to stderr.
			 */
			virtual void PrintErrorTrace() const;

		protected:

		private:
			ErrorType m_type;
			ErrorImpl* m_pImpl;
			friend class InternalError;
	};
}

#endif // PGR_FC2_ERROR_H
