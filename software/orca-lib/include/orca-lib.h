#ifndef __ORCA_LIB_H
#define __ORCA_LIB_H

//macro for including apps headers

#define IMPORT_DEF(x) "#include x"
#define IMPORT_APP(x) IMPORT_DEF("../../applications/#x/#x.h")

//apps' entry point
void app_main(void);

#endif /* __ORCA_LIB_H */