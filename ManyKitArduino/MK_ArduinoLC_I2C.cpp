// MK_ArduinoLC_I2C.cpp

#include "MK_Arduino.h"

#if defined MK_SCREEN_I2C

uint8_t bell[8] = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
uint8_t note[8] = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
uint8_t clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t duck[8] = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
uint8_t check[8] = {0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
uint8_t retarrow[8] = {0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};

//----------------------------------------------------------------------------
void MK_Arduino::_LCI2C_Init(int addr, int numCols, int numRows)
{
    if (mLiquidCrystal_I2C)
    {
        delete (mLiquidCrystal_I2C);
    }
    mLiquidCrystal_I2C = 0;
    mLiquidCrystal_I2C = new LiquidCrystal_I2C(addr, numCols, numRows);
    mLiquidCrystal_I2C->init();
    mLiquidCrystal_I2C->backlight();

    mLiquidCrystal_I2C->createChar(0, bell);
    mLiquidCrystal_I2C->createChar(1, note);
    mLiquidCrystal_I2C->createChar(2, clock);
    mLiquidCrystal_I2C->createChar(3, heart);
    mLiquidCrystal_I2C->createChar(4, duck);
    mLiquidCrystal_I2C->createChar(5, check);
    mLiquidCrystal_I2C->createChar(6, cross);
    mLiquidCrystal_I2C->createChar(7, retarrow);
    mLiquidCrystal_I2C->home();
}
//----------------------------------------------------------------------------
void MK_Arduino::_LCI2C_Do(SCREEN_I2C_DoType doType)
{
    if (doType == SCR_INIT)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->init();
        }
    }
    else if (doType == SCR_CLEAR)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->init();
        }
    }
    else if (doType == SCR_HOME)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->home();
        }
    }
    else if (doType == SCR_NO_DISPLAY)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->noDisplay();
        }
    }
    else if (doType == SCR_DISPLAY)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->display();
        }
    }
    else if (doType == SCR_NO_BLINK)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->noBlink();
        }
    }
    else if (doType == SCR_BLINK)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->blink();
        }
    }
    else if (doType == SCR_NO_CURSOR)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->noCursor();
        }
    }
    else if (doType == SCR_CURSOR)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->cursor();
        }
    }
    else if (doType == SCR_SCROOL_DISPLAYLEFT)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->scrollDisplayLeft();
        }
    }
    else if (doType == SCR_SCROOL_DISPLAYRIGHT)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->scrollDisplayRight();
        }
    }
    else if (doType == SCR_LEFT_TO_RIGHT)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->leftToRight();
        }
    }
    else if (doType == SCR_RIGHT_TO_LEFT)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->rightToLeft();
        }
    }
    else if (doType == SCR_NO_BACKLIGHT)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->noBacklight();
        }
    }
    else if (doType == SCR_BACKLIGHT)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->backlight();
        }
    }
    else if (doType == SCR_NO_AUTO_SCROOL)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->noAutoscroll();
        }
    }
    else if (doType == SCR_AUTO_SCROLL)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->autoscroll();
        }
    }
    else if (doType == SCR_BLINK_ON)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->blink_on();
        }
    }
    else if (doType == SCR_BLINK_OFF)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->blink_off();
        }
    }
    else if (doType == SCR_ON)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->on();
        }
    }
    else if (doType == SCR_OFF)
    {
        if (mLiquidCrystal_I2C)
        {
            mLiquidCrystal_I2C->off();
        }
    }
}
//----------------------------------------------------------------------------
void MK_Arduino::_LCI2C_SetCursor(int col, int row)
{
    if (mLiquidCrystal_I2C)
    {
        mLiquidCrystal_I2C->setCursor(col, row);
    }
}
//----------------------------------------------------------------------------
void MK_Arduino::_LCI2C_SetBackLight(int val)
{
    if (mLiquidCrystal_I2C)
    {
        mLiquidCrystal_I2C->setBacklight(val);
    }
}
//----------------------------------------------------------------------------
void MK_Arduino::_LCI2C_Print(String val)
{
    if (mLiquidCrystal_I2C)
    {
        mLiquidCrystal_I2C->print(val);
    }
}
//----------------------------------------------------------------------------
void MK_Arduino::_LCI2C_PrintByte(int selfCreateCharIndex)
{
    if (mLiquidCrystal_I2C)
    {
        mLiquidCrystal_I2C->write((byte)selfCreateCharIndex);
    }
}
//----------------------------------------------------------------------------

#endif
