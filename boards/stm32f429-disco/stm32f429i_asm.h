/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.   
*/
/* Assembly Language routines which expose mbed1768 board specifc functionality to the mri debugger. */
#ifndef _STM32F429DISCO_ASM_H_
#define _STM32F429I_ASM_H_

int      __mriDisableMbed(void);
int      __mriGetMbedUid(uint8_t* pOutputBuffer);

#endif /* _STM32F429DISCO_ASM_H_ */
