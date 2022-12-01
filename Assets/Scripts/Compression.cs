using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using UnityEngine;

//************************** BitPacking ***********************************
//*************************************************************************

// Limit core data to the following characters:
// 0-9.-+<sep1><sep2>
// Gives us 16 characters, allowing for 2X compression using 8-bit UTF
// This method is super fast and guarantees 50% reduction at all times.
// Further reduction by compression may be possible


static class BitPacking
{
    // Encoding and Decoding hashes
    // Not used due to performance reason, but usefull as a look-up table
    static Dictionary<char, char> hash_enc = new Dictionary<char, char>
    {
        {(char) '0',(char) 0 },
        {(char) '1',(char) 1 },
        {(char) '2',(char) 2 },
        {(char) '3',(char) 3 },
        {(char) '4',(char) 4 },
        {(char) '5',(char) 5 },
        {(char) '6',(char) 6 },
        {(char) '7',(char) 7 },
        {(char) '8',(char) 8 },
        {(char) '9',(char) 9 },
        {(char) '.',(char) 10 },
        {(char) '-',(char) 11 },
        {(char) '+',(char) 12 },
        {(char) 'e',(char)13 },
        {(char) 'E',(char) 13 },
        {(char) GLOBALS.SEPARATOR1,(char) 14 }, // Seperator 1
        {(char) GLOBALS.SEPARATOR2,(char) 15 }  // Seperator 2
    };


    // Limit the output to 20k (40k effective input for compression, 10k output for decompression)
    private static char[] buffer = new char[20000];

    public static byte[] Compress(string inputraw)
    {
        // Sanity check
        if (inputraw == null || inputraw.Length == 0 ) { return new byte[0]; }

        // If string too large, do not compress
        if (inputraw.Length > 19999) { return Encoding.UTF8.GetBytes(inputraw.ToCharArray()); }


        char[] input = inputraw.ToCharArray();
        int inputLength = input.Length;

        int level = 0; // The current packing level
        char currbyte = (char)0;// the current uncompressed byte being assembled
        int buff_length = 0;  // number of bytes in buffer 


        // Go through each uncompressed byte
        for (int i = 0; i < inputLength; i++)
        {
            char currinput = input[i];

            if (currinput >= '0' && currinput <= '9')
            {
                currbyte += (char)((currinput - '0') << (level++ * 4));
            }
            else if (currinput == '.')
            {
                currbyte += (char)((char)10 << (level++ * 4));
            }
            else if (currinput == '-')
            {
                currbyte += (char)((char)11 << (level++ * 4));
            }
            else if (currinput == '+')
            {
                currbyte += (char)((char)12 << (level++ * 4));
            }
            else if (currinput == GLOBALS.SEPARATOR1)
            {
                currbyte += (char)((char)14 << (level++ * 4));
            }
            else if (currinput == GLOBALS.SEPARATOR2)
            {
                currbyte += (char)((char)15 << (level++ * 4));
            }
            else if (currinput == 'e')
            {
                currbyte += (char)((char)13 << (level++ * 4));
            }
            else if (currinput == 'E')
            {
                currbyte += (char)((char)13 << (level++ * 4));
            }
            else
            {
                Debug.LogError("Bit packing failed on " + inputraw);
                // Invalid character detected, do not compress
                return Encoding.UTF8.GetBytes(inputraw.ToCharArray());
            }


            // Encode current byte
            // currbyte += (char) (hash_enc[input[i]] << (level++ * 4)); // Hash call is the most time expensive operation. Maybe try something else?

            // If we reached the end of our bit-packing, start a new character
            if (level == 2)
            {
                buffer[buff_length++] = currbyte;
                currbyte = (char)0;
                level = 0;
            }
        }

        // Add the last partially assembled byte (if it exists)
        if (level > 0)
        {
            // The problem here is that the second half of the byte will be decoded as 0.
            // No way of specifying it to be null, thus will chose seperator1 as the best alternative
            buffer[buff_length] = currbyte;
            buffer[buff_length++] += (char) ((char)14 << 4);  
        }

        // Now return the compressed data
        return MyUtils.CharToByteArray(buffer, buff_length);
    }

    public static byte[] Decompress(byte[] input)
    {
        // Sanity check
        if (input == null || input.Length == 0 ) { return new byte[0]; }

        // If string too large, do not compress
        if (input.Length > 9999) { return input; }

        int inputLength = input.Length;
        int outputLength = 0;

        // Go through each uncompressed byte
        for (int i = 0; i < inputLength; i++)
        {
            byte zipinput = input[i];

            for (int level = 0; level < 2; level++)
            {
                byte currinput = (level==0) ? (byte) (zipinput & 0x0F) :
                                              (byte)((zipinput >> 4) & 0x0F);

                if (currinput >= 0 && currinput <= 9)
                {
                    buffer[outputLength++] = (char) (currinput + '0');
                }
                else if (currinput == 10)
                {
                    buffer[outputLength++] = '.';
                }
                else if (currinput == 11)
                {
                    buffer[outputLength++] = '-';
                }
                else if (currinput == 12)
                {
                    buffer[outputLength++] = '+';
                }
                else if (currinput == 14)
                {
                    buffer[outputLength++] = GLOBALS.SEPARATOR1;
                }
                else if (currinput == 15)
                {
                    buffer[outputLength++] = GLOBALS.SEPARATOR2;
                }
                else if (currinput == 13)
                {
                    buffer[outputLength++] = 'e';
                }

                
            }
        }

        // Now return the compressed data
        return MyUtils.CharToByteArray(buffer, outputLength);

    }

}















//************************************** Improved LZF ******************************
/*
 * Improved version to C# LibLZF Port:
 * Copyright (c) 2010 Roman Atachiants <kelindar@gmail.com>
 *
 * Original CLZF Port:
 * Copyright (c) 2005 Oren J. Maurice <oymaurice@hazorea.org.il>
 *
 * Original LibLZF Library  Algorithm:
 * Copyright (c) 2000-2008 Marc Alexander Lehmann <schmorp@schmorp.de>
 *
 * Redistribution and use in source and binary forms, with or without modifica-
 * tion, are permitted provided that the following conditions are met:
 *
 *   1.  Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *   3.  The name of the author may not be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MER-
 * CHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPE-
 * CIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTH-
 * ERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, the contents of this file may be used under the terms of
 * the GNU General Public License version 2 (the "GPL"), in which case the
 * provisions of the GPL are applicable instead of the above. If you wish to
 * allow the use of your version of this file only under the terms of the
 * GPL and not to allow others to use your version of this file under the
 * BSD license, indicate your decision by deleting the provisions above and
 * replace them with the notice and other provisions required by the GPL. If
 * you do not delete the provisions above, a recipient may use your version
 * of this file under either the BSD or the GPL.
 */


/* Benchmark with Alice29 Canterbury Corpus
        ---------------------------------------
        (Compression) Original CLZF C#
        Raw = 152089, Compressed = 101092
         8292,4743 ms.
        ---------------------------------------
        (Compression) My LZF C#
        Raw = 152089, Compressed = 101092
         33,0019 ms.
        ---------------------------------------
        (Compression) Zlib using SharpZipLib
        Raw = 152089, Compressed = 54388
         8389,4799 ms.
        ---------------------------------------
        (Compression) QuickLZ C#
        Raw = 152089, Compressed = 83494
         80,0046 ms.
        ---------------------------------------
        (Decompression) Original CLZF C#
        Decompressed = 152089
         16,0009 ms.
        ---------------------------------------
        (Decompression) My LZF C#
        Decompressed = 152089
         15,0009 ms.
        ---------------------------------------
        (Decompression) Zlib using SharpZipLib
        Decompressed = 152089
         3577,2046 ms.
        ---------------------------------------
        (Decompression) QuickLZ C#
        Decompressed = 152089
         21,0012 ms.
    */



/// <summary>
/// Improved C# LZF Compressor, a very small data compression library. The compression algorithm is extremely fast.
public static class CompressionHelper
{
    private static readonly uint HLOG = 14;
    private static readonly uint HSIZE = (1 << 14);
    private static readonly uint MAX_LIT = (1 << 5);
    private static readonly uint MAX_OFF = (1 << 13);
    private static readonly uint MAX_REF = ((1 << 8) + (1 << 3));

    /// <summary>
    /// Hashtable, that can be allocated only once
    /// </summary>
    private static readonly long[] HashTable = new long[HSIZE];

    // Limit the output to 60k 
    private static byte[] tempBuffer = new byte[60000];

    // Compresses inputBytes
    public static byte[] Compress(string inputBytes)
    {
        // Make sure input isn't zero
        if (inputBytes == null || inputBytes.Length == 0)
        { return new byte[0]; }

        byte[] input = MyUtils.CharToByteArray(inputBytes.ToCharArray(), inputBytes.Length);
        return Compress(input);
    }

    // Compresses inputBytes
    public static byte[] Compress(byte[] inputBytes)
    {
        // Make sure input isn't zero
        if (inputBytes == null || inputBytes.Length == 0)
        { return new byte[0]; }

        // Use maximum size buffer and limit to 20k output
        int byteCount = lzf_compress(inputBytes, ref tempBuffer);

        // If byteCount is 0, then error
        if (byteCount == 0)
        {
            Debug.LogError("lzf_compress failed. Input length = " + inputBytes.Length);
            return new byte[0];
        }

        byte[] outputBytes = new byte[byteCount];
        Buffer.BlockCopy(tempBuffer, 0, outputBytes, 0, byteCount);
        return outputBytes;
    }


    // Decompress outputBytes
    public static byte[] Decompress(byte[] inputBytes)
    {
        if (inputBytes == null || inputBytes.Length == 0)
        { return new byte[0]; }

        // Use maximum size buffer and limit to 20k output
        int byteCount = lzf_decompress(inputBytes, ref tempBuffer);

        // If byteCount is 0, then error
        if (byteCount == 0)
        {
            Debug.LogError("lzf_decompress failed. Input Length = " + inputBytes.Length );
            return new byte[0];
        }

        byte[] outputBytes = new byte[byteCount];
        Buffer.BlockCopy(tempBuffer, 0, outputBytes, 0, byteCount);
        return outputBytes;
    }

    /// <summary>
    /// Compresses the data using LibLZF algorithm
    /// </summary>
    /// <param name="input">Reference to the data to compress</param>
    /// <param name="output">Reference to a buffer which will contain the compressed data</param>
    /// <returns>The size of the compressed archive in the output buffer</returns>
    public static int lzf_compress(byte[] input, ref byte[] output)
    {
        int inputLength = input.Length;
        int outputLength = output.Length;

        // Sanity checks: need at least 2 bytes in input buffer for compression
        if (inputLength == 0 || outputLength == 0)  { return 0;  }
        if (inputLength == 1 )
        {
            output[0] = (byte) input[0];
            return 1;
        }


        Array.Clear(HashTable, 0, (int)HSIZE);

        long hslot;
        uint iidx = 0;
        uint oidx = 0;
        long reference;

        uint hval = (uint)(((input[iidx]) << 8) | input[iidx + 1]); // FRST(in_data, iidx);
        long off;
        int lit = 0;

        for (; ; )
        {
            if (iidx < inputLength - 2)
            {
                hval = (hval << 8) | input[iidx + 2];
                hslot = ((hval ^ (hval << 5)) >> (int)(((3 * 8 - HLOG)) - hval * 5) & (HSIZE - 1));
                reference = HashTable[hslot];
                HashTable[hslot] = (long)iidx;


                if ((off = iidx - reference - 1) < MAX_OFF
                    && iidx + 4 < inputLength
                    && reference > 0
                    && input[reference + 0] == input[iidx + 0]
                    && input[reference + 1] == input[iidx + 1]
                    && input[reference + 2] == input[iidx + 2]
                    )
                {
                    /* match found at *reference++ */
                    uint len = 2;
                    uint maxlen = (uint)inputLength - iidx - len;
                    maxlen = maxlen > MAX_REF ? MAX_REF : maxlen;

                    if (oidx + lit + 1 + 3 >= outputLength)
                        return 0;

                    do
                        len++;
                    while (len < maxlen && input[reference + len] == input[iidx + len]);

                    if (lit != 0)
                    {
                        output[oidx++] = (byte)(lit - 1);
                        lit = -lit;
                        do
                            output[oidx++] = (byte) input[iidx + lit];
                        while ((++lit) != 0);
                    }

                    len -= 2;
                    iidx++;

                    if (len < 7)
                    {
                        output[oidx++] = (byte)((off >> 8) + (len << 5));
                    }
                    else
                    {
                        output[oidx++] = (byte)((off >> 8) + (7 << 5));
                        output[oidx++] = (byte)(len - 7);
                    }

                    output[oidx++] = (byte)off;

                    iidx += len - 1;
                    hval = (uint)(((input[iidx]) << 8) | input[iidx + 1]);

                    hval = (hval << 8) | input[iidx + 2];
                    HashTable[((hval ^ (hval << 5)) >> (int)(((3 * 8 - HLOG)) - hval * 5) & (HSIZE - 1))] = iidx;
                    iidx++;

                    hval = (hval << 8) | input[iidx + 2];
                    HashTable[((hval ^ (hval << 5)) >> (int)(((3 * 8 - HLOG)) - hval * 5) & (HSIZE - 1))] = iidx;
                    iidx++;
                    continue;
                }
            }
            else if (iidx == inputLength)
                break;

            /* one more literal byte we must copy */
            lit++;
            iidx++;

            if (lit == MAX_LIT)
            {
                if (oidx + 1 + MAX_LIT >= outputLength)
                    return 0;

                output[oidx++] = (byte)(MAX_LIT - 1);
                lit = -lit;
                do
                    output[oidx++] = (byte) input[iidx + lit];
                while ((++lit) != 0);
            }
        }

        if (lit != 0)
        {
            if (oidx + lit + 1 >= outputLength)
                return 0;

            output[oidx++] = (byte)(lit - 1);
            lit = -lit;
            do
                output[oidx++] = (byte) input[iidx + lit];
            while ((++lit) != 0);
        }

        return (int)oidx;
    }


    /// <summary>
    /// Decompresses the data using LibLZF algorithm
    /// </summary>
    /// <param name="input">Reference to the data to decompress</param>
    /// <param name="output">Reference to a buffer which will contain the decompressed data</param>
    /// <returns>Returns decompressed size</returns>
    public static int lzf_decompress(byte[] input, ref byte[] output)
    {
        int inputLength = input.Length;
        int outputLength = output.Length;

        // Sanity checks: need at least 2 bytes in input buffer for compression
        if (inputLength == 0 || outputLength == 0) { return 0; }
        if (inputLength == 1)
        {
            output[0] = input[0];
            return 1;
        }


        uint iidx = 0;
        uint oidx = 0;

        do
        {
            uint ctrl = input[iidx++];

            if (ctrl < (1 << 5)) /* literal run */
            {
                ctrl++;

                if (oidx + ctrl > outputLength)
                {
                    //SET_ERRNO (E2BIG);
                    return 0;
                }

                do
                    output[oidx++] = input[iidx++];
                while ((--ctrl) != 0);
            }
            else /* back reference */
            {
                uint len = ctrl >> 5;

                int reference = (int)(oidx - ((ctrl & 0x1f) << 8) - 1);

                if (len == 7)
                    len += input[iidx++];

                reference -= input[iidx++];

                if (oidx + len + 2 > outputLength)
                {
                    //SET_ERRNO (E2BIG);
                    return 0;
                }

                if (reference < 0)
                {
                    //SET_ERRNO (EINVAL);
                    return 0;
                }

                output[oidx++] = output[reference++];
                output[oidx++] = output[reference++];

                do
                    output[oidx++] = output[reference++];
                while ((--len) != 0);
            }
        }
        while (iidx < inputLength);

        return (int)oidx;
    }

}
