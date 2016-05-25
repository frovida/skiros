#Possible types
int8 RESOURCE = 0
int8 LITERAL = 1
int8 BLANK = 2

string uri
int8 type
#Optional: only if type==LITERAL
string literal_type

