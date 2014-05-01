WS   [ \r\n\t]+
LT   [A-Za-z]
KR   [A-Za-z0-9_\[\]\{\}/.&#$%~'@^]
DI   [0-9]
INNM {DI}+
NM   (({DI}+\.?{DI}*)|(\.{DI}+))([Ee][-+]?{DI}+)?
VR   {LT}{KR}*(<{KR}+>)?
OPLE [<]?=?
OPGE [>]=?
EOW  $|[*+-;<=>]|{WS}
INF  [ \r\n\t]*[-+][Ii][Nn][Ff]([Ii][Nn][Ii][Tt]([Ee]|[Yy]))?/{EOW}
S_OP [-+ \t\n\r]+
MIN  [mM][iI][nN]([iI][mM][iI][zZsS][eE])?[ \t]*:
MAX  [mM][aA][xX]([iI][mM][iI][zZsS][eE])?[ \t]*:
INT  ^[ \t]*[Ii][Nn][Tt]([Ee][Gg][Ee][Rr])?
BIN  ^[ \t]*[Bb][Ii][Nn]([Aa][Rr][Yy])?
SEC  ^[ \t]*([Ss][Ee][Cc])|([Ss][Ee][Mm][Ii]-[Cc][Oo][Nn][Tt][Ii][Nn][Uu][Oo][Uu][Ss])|([Ss][Ee][Mm][Ii])|([Ss][Ee][Mm][Ii][Ss])
SOS  ^[ \t]*([Ss][Oo][Ss][12]*)|([Ss][Ee][Tt][Ss])
FREE ^[ \t]*[Ff][Rr][Ee][Ee]
LBL  {VR}:
SOSD {LBL}:

%start COMMENT
%start LINECOMMENT

%%
<INITIAL>"/*" {
  BEGIN COMMENT;
} /* begin skip comment */

<COMMENT>"*/" {
  BEGIN INITIAL;
} /* end skip comment */

<COMMENT>. {
}

<COMMENT>\n {
}

<COMMENT>\r {
}

<INITIAL>"//" {
  BEGIN LINECOMMENT;
} /* begin skip LINECOMMENT */

<LINECOMMENT>\n {
  BEGIN INITIAL;
} /* end skip LINECOMMENT */

<LINECOMMENT>\r {
  BEGIN INITIAL;
} /* end skip LINECOMMENT */

<LINECOMMENT>. {
}

<INITIAL>{WS} {
}

<INITIAL>"," {
  return(COMMA);
}

<INITIAL>{MIN} {
  return(MINIMISE);
}

<INITIAL>{MAX} {
  return(MAXIMISE);
}

<INITIAL>{INNM} {
  f = atof((char *)yytext);
  return(INTCONS);
} /* f contains the last float */

<INITIAL>{NM} {
  f = atof((char *)yytext);
  return(CONS);
} /* f contains the last float */

<INITIAL>{INF} {
  char *ptr;

  f = DEF_INFINITE;
  Sign = 0;
  ptr = (char *)yytext;
  while (isspace(*ptr)) ptr++;
  if(*ptr == '-')
    Sign = 1;
  return(INF);
} /* f contains the last float */

<INITIAL>{S_OP} {
  Sign = 0;
  for(x = 0; x < yyleng; x++)
    if(yytext[x] == '-' || yytext[x] == '+')
      Sign = (Sign == (yytext[x] == '+'));
  return (SIGN);
  /* Sign is TRUE if the sign-string
     represents a '-'. Otherwise Sign
     is FALSE */
}

<INITIAL>{INT} {
  if((!Within_int_decl) && (!Within_sec_decl) && (!Within_sos_decl) && (!Within_free_decl)) {
    Within_int_decl = 1;
    Within_sos_decl1 = FALSE;
  }
  return(SEC_INT);
}

<INITIAL>{BIN} {
  if((!Within_int_decl) && (!Within_sec_decl) && (!Within_sos_decl) && (!Within_free_decl)) {
    Within_int_decl = 2;
    Within_sos_decl1 = FALSE;
  }
  return(SEC_BIN);
}

<INITIAL>{SEC} {
  if((!Within_int_decl) && (!Within_sec_decl) && (!Within_sos_decl) && (!Within_free_decl)) {
    Within_sec_decl = TRUE;
    Within_sos_decl1 = FALSE;
  }
  return(SEC_SEC);
}

<INITIAL>{SOS} {
  if(!Within_sos_decl)
    SOStype0 = (short)atoi(((char *)yytext) + 3);
  if((!Within_int_decl) && (!Within_sec_decl) && (!Within_sos_decl) && (!Within_free_decl))
    Within_sos_decl = TRUE;
  return(SEC_SOS);
}

<INITIAL>{SOSD} {
  FREE(Last_var);
  Last_var = strdup((char *)yytext);
  Last_var[strlen(Last_var) - 2] = 0;
  return(SOSDESCR);
}

<INITIAL>{FREE} {
  if((!Within_int_decl) && (!Within_sec_decl) && (!Within_sos_decl) && (!Within_free_decl)) {
    Within_free_decl = TRUE;
    Within_sos_decl1 = FALSE;
  }
  return(SEC_FREE);
}

<INITIAL>{LBL} {
  FREE(Last_var);
  Last_var = strdup((char *)yytext);
  Last_var[strlen(Last_var) - 1] = 0;
  return(VARIABLECOLON);
}

<INITIAL>{VR} {
  FREE(Last_var);
  Last_var = strdup((char *)yytext);
  return(VAR);
}

<INITIAL>":" {
  return (COLON);
}

<INITIAL>"*" {
  return(AR_M_OP);
}

<INITIAL>{OPLE} {
  return(RE_OPLE);
}

<INITIAL>{OPGE} {
  return(RE_OPGE);
}

<INITIAL>";" {
  Within_int_decl = Within_sec_decl = Within_sos_decl = Within_free_decl = FALSE;
  check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, Within_sos_decl, Within_free_decl);
  return(END_C);
}

<INITIAL>. {
  report(NULL, CRITICAL, "LEX ERROR : %s lineno %d\n", yytext, yylineno);
  return(UNDEFINED);
}

%%
