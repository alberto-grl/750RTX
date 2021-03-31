
#include "main.h"
#include <arm_const_structs.h>
#include "Presets.h"
#include "Globals.h"
#include "usbd_cdc_if.h"

int audio = 1;            // will store the value we read on this pin

int LCDline = 1;          // keeps track of which line we're printing on
int lineEnd = 21;         // One more than number of characters across display
int letterCount = 0;      // keeps track of how may characters were printed on the line
int lastWordCount = 0;    // keeps track of how may characters are in the current word
int lastSpace = 0;        // keeps track of the location of the last 'space'

// The next line stores the text that we are currently printing on a line,
// The characters in the current word,
// Our top line of text,
// Our second line of text,
// and our third line of text
// For a 20x4 display these are all 20 characters long
char currentLine[] = "12345678901234567890";
char    lastWord[] = "                    ";
char       line1[] = "                    ";
char       line2[] = "                    ";
char       line3[] = "                    ";

uint8_t ditOrDah = true;  // We have either a full dit or a full dah
int dit = 10;             // We start by defining a dit as 10 milliseconds

// The following values will auto adjust to the sender's speed
int averageDah = 240;             // A dah should be 3 times as long as a dit
int averageWordGap = 240;  // will auto adjust
long fullWait = 6000;             // The time between letters
long waitWait = 6000;             // The time between dits and dahs
long newWord = 0;                 // The time between words

uint8_t characterDone = true; // A full character has been sent

int downTime = 0;        // How long the tone was on in milliseconds
int upTime = 0;          // How long the tone was off in milliseconds
int myBounce = 2;        // Used as a short delay between key up and down

long startDownTime = 0;  // Arduino's internal timer when tone first comes on
long startUpTime = 0;    // Arduino's internal timer when tone first goes off

long lastDahTime = 0;    // Length of last dah in milliseconds
long lastDitTime = 0;    // Length oflast dit in milliseconds
long averageDahTime = 0; // Sloppy Average of length of dahs

uint8_t justDid = true; // Makes sure we only print one space during long gaps

int myNum = 0;           // We will turn dits and dahs into a binary number stored here

/////////////////////////////////////////////////////////////////////////////////
// Now here is the 'Secret Sauce'
// The Morse Code is embedded into the binary version of the numbers from 2 - 63
// The place a letter appears here matches myNum that we parsed out of the code
// #'s are miscopied characters
char mySet[] ="##TEMNAIOGKDWRUS##QZYCXBJP#L#FVH09#8###7#####/-61#######2###3#45";
char lcdGuy = ' ';       // We will store the actual character decoded here

/////////////////////////////////////////////////////////////////////////////////



 void DecodeCW(void)
 {

   if (CWIn) keyIsDown();       // LOW, or 0, means tone is being decoded
   if (!CWIn) keyIsUp();          // HIGH, or 1, means no tone is there
 }

 void keyIsDown(void) {
   // The decoder is detecting our tone

	 LED_GREEN_OFF;


   if (startUpTime>0){
     // We only need to do once, when the key first goes down
     startUpTime=0;    // clear the 'Key Up' timer
     }
   // If we haven't already started our timer, do it now
   if (startDownTime == 0){
       startDownTime = HAL_GetTick();  // get Arduino's current clock time
      }

     characterDone=false; // we're still building a character
     ditOrDah=false;      // the key is still down we're not done with the tone
//TODO check if it is really needed     delay(myBounce);     // Take a short breath here

   if (myNum == 0) {      // myNum will equal zero at the beginning of a character
      myNum = 1;          // This is our start bit  - it only does this once per letter
      }
 }

  void keyIsUp() {
   // The decoder does not detect our tone

	  LED_GREEN_ON;

   // If we haven't already started our timer, do it now
   if (startUpTime == 0){startUpTime = HAL_GetTick();}

   // Find out how long we've gone with no tone
   // If it is twice as long as a dah print a space
   upTime = HAL_GetTick() - startUpTime;
   if (upTime<10)return;
   if (upTime > (averageDah*2)) {
      printSpace();
   }

   // Only do this once after the key goes up
   if (startDownTime > 0){
     downTime = HAL_GetTick() - startDownTime;  // how long was the tone on?
     startDownTime=0;      // clear the 'Key Down' timer
   }

   if (!ditOrDah) {
     // We don't know if it was a dit or a dah yet
      shiftBits();    // let's go find out! And do our Magic with the bits
    }

    // If we are still building a character ...
    if (!characterDone) {
       // Are we done yet?
       if (upTime > dit) {
         // BINGO! we're done with this one
         printCharacter();       // Go figure out what character it was and print it
         characterDone=true;     // We got him, we're done here
         myNum=0;                // This sets us up for getting the next start bit
         }
         downTime=0;               // Reset our keyDown counter
       }
   }


void shiftBits() {
  // we know we've got a dit or a dah, let's find out which
  // then we will shift the bits in myNum and then add 1 or not add 1

  if (downTime < dit / 3) return;  // ignore my keybounce

  myNum = myNum << 1;   // shift bits left
  ditOrDah = true;        // we will know which one in two lines


  // If it is a dit we add 1. If it is a dah we do nothing!
  if (downTime < dit) {
     myNum++;           // add one because it is a dit
     } else {

    // The next three lines handle the automatic speed adjustment:
    averageDah = (downTime+averageDah) / 2;  // running average of dahs
    dit = averageDah / 3;                    // normal dit would be this
    dit = dit * 2;    // double it to get the threshold between dits and dahs
     }
  }


void printCharacter() {
  justDid = false;         // OK to print a space again after this

  // Punctuation marks will make a BIG myNum
  if (myNum > 63) {
    printPunctuation();  // The value we parsed is bigger than our character array
                         // It is probably a punctuation mark so go figure it out.
    return;              // Go back to the main loop(), we're done here.
  }
  lcdGuy = mySet[myNum]; // Find the letter in the character set
  sendToLCD();           // Go figure out where to put in on the display
}

void printSpace() {
  if (justDid) return;  // only one space, no matter how long the gap
  justDid = true;       // so we don't do this twice

  // We keep track of the average gap between words and bump it up 20 milliseconds
  // do avoid false spaces within the word
  averageWordGap = ((averageWordGap + upTime) / 2) + 20;

  lastWordCount=0;      // start counting length of word again
  currentLine[letterCount]=' ';  // and a space to the variable that stores the current line
  lastSpace=letterCount;         // keep track of this, our last, space

  // Now we need to clear all the characters out of our last word array
  for (int i=0; i<20; i++) {
    lastWord[i]=' ';
   }

  lcdGuy=' ';            // this is going to go to the LCD

  // We don't need to print the space if we are at the very end of the line
  if (letterCount < 20) {
    sendToLCD();         // go figure out where to put it on the display
 }
}

void printPunctuation() {
  // Punctuation marks are made up of more dits and dahs than
  // letters and numbers. Rather than extend the character array
  // out to reach these higher numbers we will simply check for
  // them here. This funtion only gets called when myNum is greater than 63

  // Thanks to Jack Purdum for the changes in this function
  // The original uses if then statements and only had 3 punctuation
  // marks. Then as I was copying code off of web sites I added
  // characters we don't normally see on the air and the list got
  // a little long. Using 'switch' to handle them is much better.


  switch (myNum) {
    case 71:
      lcdGuy = ':';
      break;
    case 76:
      lcdGuy = ',';
      break;
    case 84:
      lcdGuy = '!';
      break;
    case 94:
      lcdGuy = '-';
      break;
    case 97:
      lcdGuy = 39;    // Apostrophe
      break;
    case 101:
      lcdGuy = '@';
      break;
    case 106:
      lcdGuy = '.';
      break;
    case 115:
      lcdGuy = '?';
      break;
    case 246:
      lcdGuy = '$';
      break;
    case 122:
      lcdGuy = 's';
      sendToLCD();
      lcdGuy = 'k';
      break;
    default:
      lcdGuy = '#';    // Should not get here
      break;
  }
  sendToLCD();    // go figure out where to put it on the display
}

void sendToLCD(){
  // Do this only if the character is a 'space'
  if (lcdGuy > ' '){
   lastWord[lastWordCount] = lcdGuy; // store the space at the end of the array
   if (lastWordCount < lineEnd - 1) {
     lastWordCount++;   // only bump up the counter if we haven't reached the end of the line
   }
  }
  currentLine[letterCount] = lcdGuy; // now store the character in our current line array

  letterCount++;                     // we're counting the number of characters on the line

  // If we have reached the end of the line we will go do some chores
  if (letterCount == lineEnd) {
    newLine();  // check for word wrap and get ready for the next line
    return;     // so we don't need to do anything more here
  }

  // print our character at the current cursor location
  sprintf((char*)UartTXString, "%c", lcdGuy);
  	PrintUI(UartTXString);

}

//////////////////////////////////////////////////////////////////////////////////////////
// The following functions handle word wrapping and line scrolling for a 4 line display //
//////////////////////////////////////////////////////////////////////////////////////////

void newLine() {
  // sendToLCD() will call this routine when we reach the end of the line
  if (lastSpace == 0){
    // We just printed an entire line without any spaces in it.
    // We cannot word wrap this one so this character has to go at
    // the beginning of the next line.

    // First we need to clear all the characters out of our last word array
    for (int i=0; i<20; i++) {
      lastWord[i]=' ';
     }

     lastWord[0]=lcdGuy;  // store this character in the first position of our next word
     lastWordCount=1;     // set the length to 1
   }

  truncateOverFlow();    // Trim off the first part of a word that needs to go on the next line
  linePrep();            // Store the current line so we can move it up later
  reprintOverFlow();     // Print the truncated text and space padding on the next line
  }

void truncateOverFlow(){
  // Our word is running off the end of the line so we will
  // chop it off at the last space and put it at the beginning of the next line

  if (lastSpace==0) {return;}  // Don't do this if there was no space in the last line

  // Move the cursor to the place where the last space was printed on the current line
  //lcd.setCursor(lastSpace,LCDline);
  sprintf((char*)UartTXString, "\e[%d;%dH", LCDline + 9, lastSpace);
  	PrintUI(UartTXString);

  letterCount = lastSpace;    // Change the letter count to this new shorter length

  // Print 'spaces' over the top of all the letters we don't want here any more
  for (int i = lastSpace; i < 20; i++) {
    // lcd.print(' ');         // This space goes on the display
	   sprintf((char*)UartTXString, " ");
	  	PrintUI(UartTXString);

	  currentLine[i] = ' ';   // This space goes in our array
  }
}


void linePrep(){
     LCDline++;           // This is our line number, we make it one higher

     // What we do next depends on which line we are moving to
     // The first three cases are pretty simple because we working on a cleared
     // screen. When we get to the bottom, though, we need to do more.
     switch (LCDline) {
     case 1:
       // We just finished line 0
       // don't need to do anything because this for the top line
       // it is going to be thrown out when we scroll anyway.
       break;
     case 2:
       // We just finished line 1
       // We are going to move the contents of our current line into the line1 array
       for (int j=0; j<20; j++){
         line1[j] = currentLine[j];
       }
        break;
     case 3:
       // We just finished line 2
       // We are going to move the contents of our current line into the line2 holding bin
       for (int j=0; j<20; j++){
         line2[j] = currentLine[j];
       }
       break;
     case 4:
       // We just finished line 3
       // We are going to move the contents of our current line into the line3 holding bin
       for (int j=0; j<20; j++){
         line3[j] = currentLine[j];
       }
       //This is our bottom line so we will keep coming back here
       LCDline = 3;  //repeat this line over and over now. There is no such thing as line 4

       myScroll();  //move everything up a line so we can do the bottom one again
       break;
   }

}

void myScroll(){
  // We will move each line of text up one row

  int i = 0;  // we will use this variables in all our for loops

 // lcd.setCursor(0,0);      // Move the cursor to the top left corner of the display
  sprintf((char*)UartTXString, "\e[%d;%dH", 9, 0);
  	PrintUI(UartTXString);

 // lcd.print(line1);        // Print line1 here. Line1 is our second line,
                           // our top line is line0 ... on the next scroll
                           // we toss this away so we don't store line0 anywhere
    sprintf((char*)UartTXString, "%s", line1);
    	PrintUI(UartTXString);
  // Move everything stored in our line2 array into our line1 array
  for (i = 0; i < 20; i++) {
    line1[i] = line2[i];
  }

 // lcd.setCursor(0,1);      // Move the cursor to the beginning of the second line
  sprintf((char*)UartTXString, "\e[%d;%dH", 10, 0);
  	PrintUI(UartTXString);
 // lcd.print(line1);        // Print the new line1 here
  sprintf((char*)UartTXString, "%s", line1);
  	PrintUI(UartTXString);
  // Move everything stored in our line3 array into our line2 array
  for (i = 0; i < 20; i++) {
    line2[i]=line3[i];
  }
  //  lcd.setCursor(0,2);      // Move the cursor to the beginning of the third line
  sprintf((char*)UartTXString, "\e[%d;%dH", 11, 0);
  	PrintUI(UartTXString);
  //  lcd.print(line2);        // Print the new line2 here
    sprintf((char*)UartTXString, "%s", line2);
    	PrintUI(UartTXString);

  // Move everything stored in our currentLine array into our line3 array
  for (i = 0; i < 20; i++) {
    line3[i] = currentLine[i];
  }

}

void reprintOverFlow(){
  // Here we put the word that wouldn't fit at the end of the previous line
  // Back on the display at the beginning of the new line

  // Load up our current line array with what we have so far
   for (int i = 0; i < 20; i++) {
     currentLine[i] = lastWord[i];
  }
 // lcd.setCursor(0, LCDline);              // Move the cursor to the beginning of our new line
  sprintf((char*)UartTXString, "\e[%d;%dH", LCDline +9, 0);
  	PrintUI(UartTXString);
 // lcd.print(lastWord);                    // Print the stuff we just took off the previous line
  sprintf((char*)UartTXString, "%s", lastWord);
  	PrintUI(UartTXString);
  letterCount = lastWordCount;            // Set up our character counter to match the text
  //lcd.setCursor(letterCount, LCDline);
  sprintf((char*)UartTXString, "\e[%d;%dH", LCDline +9, letterCount);
   	PrintUI(UartTXString);

  lastSpace=0;          // clear the last space pointer
  lastWordCount=0;      // clear the last word length
}



