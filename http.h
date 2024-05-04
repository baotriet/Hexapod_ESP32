#ifndef HTTP_H_   /* Include guard */
#define HTTP_H_

void http_init();
byte *http_inputData();
bool http_dataReady();
void http_setDataReady(bool dataStatus);
void http_resetInputData();

#endif // HTTP_H_