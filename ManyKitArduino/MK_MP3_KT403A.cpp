// MK_MP3_KT403A.cpp

#include "MK_MP3_KT403A.h"

#if defined MK_MP3

volatile uint8_t MP3::MP3_RxPin = 0;
volatile uint8_t MP3::MP3_TxPin = 0;
//----------------------------------------------------------------------------
MP3::MP3()
{
}
//----------------------------------------------------------------------------
MP3::MP3(uint8_t rxPin, uint8_t txpin, bool inverse_logic) : MK_Serial(rxPin, txpin, inverse_logic)
{
  MP3_RxPin = rxPin;
  MP3_TxPin = txpin;
}
//----------------------------------------------------------------------------
void MP3::begin(long speed)
{
  MK_Serial::begin(speed);

  pinMode(MP3_RxPin, INPUT);
  pinMode(MP3_TxPin, OUTPUT);
}
//----------------------------------------------------------------------------
/**************************************************************** 
 * Function Name: SelectPlayerDevice
 * Description: Select the player device, U DISK or SD card.
 * Parameters: 0x01:U DISK;  0x02:SD card
 * Return: none
****************************************************************/ 
void MP3::SelectPlayerDevice(uint8_t device)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x09);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(device);
    MK_Serial::write(0xEF);
    delay(200);
}
/*
void MP3::Specify_FolderMP3_Play(uint16_t index)
{
    // uint8_t hbyte, lbyte;
    // hbyte = index / 256;
    // lbyte = index % 256;
    // MK_Serial::write(0x7E);
    // MK_Serial::write(0xFF);
    // MK_Serial::write(0x06);
    // MK_Serial::write(0x03);
    // MK_Serial::write(uint8_t(0x00));
    // MK_Serial::write(uint8_t(hbyte));
    // MK_Serial::write(uint8_t(lbyte));
    // MK_Serial::write(0xEF);
    // delay(10);
  
    uint8_t hbyte, lbyte;
    hbyte = index / 256;
    lbyte = (index+256) % 256;
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    
    MK_Serial::write(0x12);
    
    MK_Serial::write(0x00);
    
    MK_Serial::write(0x00);
    MK_Serial::write(0x01);
    MK_Serial::write(0xEF);
    delay(10);
}
*/
// 閹稿洤鐣炬稉锟�"01"閻ㄥ嫭鏋冩禒璺恒仚閿涘本娲搁惄顔昏礋"001sdafas.mp3"
void MP3::PlayFolderIndex(uint8_t folder, uint8_t index)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    
    MK_Serial::write(0x0F);
    
    MK_Serial::write(uint8_t(0x00));
    
    MK_Serial::write(uint8_t(folder));
    MK_Serial::write(uint8_t(index));
    
    MK_Serial::write(0xEF);
    delay(10);
}
/**************************************************************** 
 * Function Name: PlayPause
 * Description: Pause the MP3 player.
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::PlayPause(void)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x0E);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(0xEF);
    delay(20);
}
/**************************************************************** 
 * Function Name: Play
 * Description: Resume the MP3 player.
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::Play(void)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x0D);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(0xEF);
    delay(20);
}
//7E FF 06 16 00 00 00 FE E5 EF
void MP3::PlayStop(void)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x16);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(0xEF);
    delay(20);
}
/**************************************************************** 
 * Function Name: PlayNext
 * Description: Play the next song.
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::PlayNext(void)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x01);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(0xEF);
    delay(10);
}
/**************************************************************** 
 * Function Name: PlayPrevious
 * Description: Play the previous song.
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::PlayPrevious(void)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x02);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(0xEF);
    delay(10);
}

void MP3::PlayRandom()
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x18);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    MK_Serial::write(0xFE);
    delay(10);
}

/**************************************************************** 
 * Function Name: PlayLoop
 * Description: single song loop
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::PlaySingleLoopOpen()//7E FF 06 19 00 00 00 FE E2 EF
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x19);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    MK_Serial::write(0xFE);
    MK_Serial::write(0xE2);
    MK_Serial::write(0xEF);
    delay(10);
}
/**************************************************************** 
 * Function Name: PlayLoop
 * Description: single song stop loop
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::PlaySingleLoopClose()//7E FF 06 19 00 00 01 FE E1 EF
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x19);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    MK_Serial::write(0x01);
    MK_Serial::write(0xFE);
    MK_Serial::write(0xE1);
    MK_Serial::write(0xEF);
    delay(10);
}
/**************************************************************** 
 * Function Name: PlayLoop
 * Description: Play all the songs and loop.
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::PlayAllLoopOpen(void)//7E FF 06 11 00 00 01 EF
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x11);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    MK_Serial::write(0x01);
    MK_Serial::write(0xEF);
    delay(10);
}
/**************************************************************** 
 * Function Name: PlayLoop
 * Description: Play loop for all the songs.
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::PlayAllLoopClose(void)//7E FF 06 11 00 00 00 EF
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x11);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    MK_Serial::write(0xEF);
    delay(10);
}
void MP3::PlayFolderLoopOpen(uint8_t folder)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x17);
    MK_Serial::write(0x00);
    MK_Serial::write(0x00);
    
    MK_Serial::write((uint8_t)folder);

    MK_Serial::write(0xEF);
    delay(10);
}
/**************************************************************** 
 * Function Name: SetVolume
 * Description: Set the volume, the range is 0x00 to 0x1E.
 * Parameters: volume: the range is 0x00 to 0x1E.
 * Return: none
****************************************************************/ 
void MP3::SetVolume(uint8_t volume)
{
    if(volume>30)
        {volume=30;}
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x06);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(volume);
    MK_Serial::write(0xEF);
    delay(10);
}
/**************************************************************** 
 * Function Name: IncreaseVolume
 * Description: Increase the volume.
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::IncreaseVolume(void)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x04);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(0xEF);
    delay(10);
}
/**************************************************************** 
 * Function Name: DecreaseVolume
 * Description: Decrease the volume.
 * Parameters: none
 * Return: none
****************************************************************/ 
void MP3::DecreaseVolume(void)
{
    MK_Serial::write(0x7E);
    MK_Serial::write(0xFF);
    MK_Serial::write(0x06);
    MK_Serial::write(0x05);
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(uint8_t(0x00));
    MK_Serial::write(0xEF);
    delay(10);
}

#endif
