#ifndef _MENU_H_
#define _MENU_H_

#include "head.h"

typedef struct
{
    int menu_id;
    char menu_name[30];
    void (*menu_action)(void);
    float *param;
} menu_item;

extern float row_line;
extern float column_line;
extern int change_mode;
extern menu_item *current_menu_item;
extern menu_item menu[];

void Menu_Switch(void);
void Read_Parameter(void);
void Save_Parameter(void);
void Clear_All_Parameter(void);
int show_sub_menu(menu_item menu[], int sz, int parent_id, int highlight_col);

void printFunc(int paraCnt, ...);
#endif /* CODE_MENU_H_ */
