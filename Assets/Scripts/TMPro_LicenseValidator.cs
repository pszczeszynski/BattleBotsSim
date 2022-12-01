using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// *********************************
// Custom input validators
// *********************************
[CreateAssetMenu(fileName = "TMPro_LicenseValidator", menuName = "TMPro License Entry Validator")]
public class TMPro_LicenseValidator : TMPro.TMP_InputValidator
{
    public override char Validate(ref string text, ref int pos, char ch)
    {
        // If char isn't a number or letter, or the string is fully formed then exit
        if (!System.Char.IsDigit(ch) && !System.Char.IsLetter(ch) || pos >= 19) { return '\0'; }

        // If letter, make capital
        if (System.Char.IsLetter(ch)) { ch = System.Char.ToUpper(ch); }

        // if the count is 4,9,14 then add a '-'
        if ((text.Length == 4) || (text.Length == 9) || (text.Length == 14))
        {
            text += "-";
        }

        // If the new position lies on top of a seperator, advance it one more
        if ((pos == 4) || (pos == 9) || (pos == 14))
        {
            pos++;
        }

        // Add the char
        if (pos < text.Length)
        {
            text = text.Insert(pos++, ch.ToString());
        }
        else
        {
            text += ch.ToString();
            pos++;
        }

        // If this is an insertion, we want to replace the char
        if (pos <= text.Length - 1)
        {
            
            text = text.Remove(pos, 1);
            return ch;
        }

        // if the count is 4,9,14 then add a '-'
        if ((text.Length == 4) || (text.Length == 9) || (text.Length == 14))
        {
            text += "-";
        }

        return ch;
    }

}