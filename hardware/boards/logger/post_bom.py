import pandas as pd

df=pd.read_csv('logger_01_bom.csv')
df.columns = [s.strip() for s in df.columns]
df=df.set_index('Reference')

# for digikey - needs a quantity field, too.

print "Parts with no BOM info:"
print df.loc[ df.part_num.isnull(),['part_num','digikey_num','Value']]

##

# drop parts with no part numbers
df2=df[ ~( df.part_num.isnull() & df.digikey_num.isnull()) ]

def concat(refs):
    return " ".join( refs.values )
    
df3=df2.reset_index().groupby(['part_num']).agg( {'Value':'count',
                                                  'digikey_num':'first',
                                                  'Reference':concat} )
df3.rename( columns={'Value':'Quantity'},inplace=True)

print "Parts missing a digikey number:"
print df3[ df3.digikey_num.isnull() ]

print "-"*70

print df3

df3.to_csv('post_bom.csv')


