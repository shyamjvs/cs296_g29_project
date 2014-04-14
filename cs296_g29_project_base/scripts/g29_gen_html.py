######### A few string helper functions ###################

alphabets = [chr(i) for i in range(ord('a'), ord('z') + 1)]

token_equivalent = {}
token_equivalent["section"] = "h2"
token_equivalent["subsection"] = "h3"
token_equivalent["subsubsection"] = "h4"
token_equivalent["title"] = "h1"
token_equivalent["textbf"] = "strong"
token_equivalent["caption"] = "figcaption"


def has_alphabet(line):
    has_alpha = False
    for i in range(0,len(line)):
        if line[i] in alphabets:
            has_alpha = True
            break
    return has_alpha

###########################################################

def get_token(line):

    line_s = " ".join(line.split())
    token = ""
    if line_s[0] == chr(92):
        if line_s.find("{") != -1:
            token = line_s[1:line_s.find("{")]
        else:
            token = line_s[1:]
    if token == "":
        line_part = line_s
    elif token.find("author") != -1 or token.find("and") != -1:
        line_part = ""
    elif token.find("begin") != -1 or token.find("end") != -1:
        line_part = ""
    else:
        if line_s.find("{") != -1:
            line_part = line_s[line_s.find("{")+1:line_s.rfind("}")]
        else:
            line_part = ""

    pair_of_parts = [ token , line_part ]
    return pair_of_parts

def modify(line,citation_count):

    bf_index = line.find("\\textbf{")
    cite_index = line.find("\\cite{")
    
    if bf_index != -1:
        line = line.replace("\\textbf{","<strong>")
        line = line.replace("}","</strong>")
    if cite_index != -1:
        line = line.replace("\\cite{","[")
        line = line.replace("}","]")
        line = line.replace(line[line.find("["):line.find("]")+1], "["+str(citation_count)+"]")
        citation_count = citation_count + 1
    return line
    
def convert(line, citation_count, want_newline):

    if("".join(line.split()) == ""):
        return "\n"

    line.replace("\n","")
    token = get_token(line)
    token[1] = token[1].replace("\\\\","<br>")
    if token[0] == "":
        if has_alphabet(token[1]) == True:
            new_line = modify(token[1],citation_count)
        else:
            new_line = "</div>\n<br><br>"
    elif token[0].find("maketitle") != -1 or token[0].find("date")!=-1:
        new_line = ""
    elif token[0].find("includegraphics") != -1:
        new_line = '<center>\n<img width="800" style="padding:0.5cm" src= "' + token[1] + '.png'+'">\n</center>'
    elif token[0].find("begin") != -1:
        new_line = "<div>"
    elif token[0].find("end") != -1:
        new_line = "</div>"
    elif token[0] == "author":
        new_line = '<div style="margin-right:45px; float:left">'
    elif token[0] == "and":
        new_line = '</div>\n<div style="margin-right:45px; float:left">'
    elif token[0] == "texttt":
        new_line = ""
    elif token[0] == "bibliographystyle":
        new_line = ""
    elif token[0] == "centering":
        new_line = ""
    else:
        new_line = "<" + token_equivalent[token[0]] + "> "
        new_line = new_line + convert(token[1], citation_count, False) 
        new_line = new_line + " </" + token_equivalent[token[0]] + ">"
    
    if want_newline == True:
        return new_line.replace("\\","") + "\n"
    else:
        return new_line.replace("\\","")

######### Code for the parser ############################

import re
f = open("../doc/g29_project_report.tex")
f2 = open("../doc/g29_project_report.html",'w')

f2.write("<!DOCTYPE html>\n")
f2.write("<html>\n")

########## Part for the html head and styling #############

f2.write("<head>\n")
f2.write("<title> Group-29 CS296 Project Report </title>\n")
f2.write("</head>\n\n")

f2.write("<body>\n")

###### Put body here ######################################

lines = f.readlines()
citation_count = 1    
for i in range(7,158): 
    new_line = convert(lines[i],citation_count,True)
    f2.write(new_line)

###############################
f2.write("<br><br>\n")
f2.write("<h2>References</h2>\n")
f2.write("<div>[1] : <a href= 'http://en.wikipedia.org/wiki/Bulldozer/'>http://en.wikipedia.org/wiki/Bulldozer/</a> <br>\n")
f2.write("[2] : <a href= 'http://www.box2d.org/manual.html'>http://www.box2d.org/manual.html</a> <br>\n")
f2.write("[3] : <a href='http://matplotlib.org/'>http://matplotlib.org/</a> <br>\n")
f2.write("[4] : <a href='http://gprof2dot.jrfonseca.googlecode.com/git/gprof2dot.py'>http://gprof2dot.jrfonseca.googlecode.com/git/gprof2dot.py</a> <br>\n")
f2.write("<br><br>\n")
f2.write("<a href='http://www.cse.iitb.ac.in/~jvs/project.html'>Link to the project page</a>\n")
f2.write("</body>\n")
f2.write("</html>\n")
f2.close()
