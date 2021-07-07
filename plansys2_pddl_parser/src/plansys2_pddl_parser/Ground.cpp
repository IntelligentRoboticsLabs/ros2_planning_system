
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

Ground::Ground( const Ground * g, Domain & d )
	: ParamCond( g ), lifted( d.preds.get( g->name ) ) {}

void Ground::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( " << name;
	for ( unsigned i = 0; i < params.size(); ++i ) {
		if ( ts.size() && params[i] >= 0 && (unsigned)params[i] < ts.size() ) s << " " << ts[params[i]];
		else if (params[i] >= 0 && (unsigned)params[i] >= ts.size()) s << " ?" << params[i]; 
		else s << " " << d.types[lifted->params[i]]->object( params[i] ).first;
	}
	s << " )";
}

plansys2_msgs::msg::Node::SharedPtr Ground::getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace ) const {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    if ( d.funcs.index( name ) >= 0) {
        node->node_type = plansys2_msgs::msg::Node::FUNCTION;
    } else {
        node->node_type = plansys2_msgs::msg::Node::PREDICATE;
    }
    node->node_id = tree.nodes.size();
    node->name = name;
    for ( unsigned i = 0; i < params.size(); ++i ) {
        plansys2_msgs::msg::Param param;
        if (i < replace.size()){
          if (params[i] > 0)  // param has a variable value; replace by action-args
          {
            param.name = replace[params[i]];
          }
           else                // param has a constant value; retrive from domain::type[t_i]::constants[c_i]
          {
            int type_idx = lifted->params[i];            // idx of the type of this param [ref: d.type]
            int constant_idx = (-1 * params[i]) -1;      // idx of the constant value [ref: d.type.constant]
            param.name = d.types[type_idx]->constants[constant_idx]; // the actual constant value
          }
        } else if (d.types[lifted->params[i]]->objects.size() > params[i]) {
          param.name = d.types[lifted->params[i]]->object( params[i] ).first;
        } else {
          param.name = "?" + std::to_string(params[i]);
        }
        param.type = d.types[lifted->params[i]]->name;
        node->parameters.push_back(param);
    }
    tree.nodes.push_back(*node);
    return node;
}

void Ground::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	params.resize( lifted->params.size() );
	for ( unsigned i = 0; i < lifted->params.size(); ++i, f.next() ) {
		std::string s = f.getToken();
		int k = ts.index( s );
		if ( k >= 0 ) params[i] = k;
		else {
			std::pair< bool, int > p = d.types[lifted->params[i]]->parseConstant( s );
			if ( p.first ) params[i] = p.second;
			else f.tokenExit( s );
		}
	}
	f.assert_token( ")" );
}

} } // namespaces
