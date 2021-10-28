package ru.armbot.service

import builders.dsl.spreadsheet.api.FontStyle
import builders.dsl.spreadsheet.api.Keywords;
import builders.dsl.spreadsheet.builder.api.CanDefineStyle;
import builders.dsl.spreadsheet.builder.api.Stylesheet
import org.apache.poi.xwpf.usermodel.VerticalAlign;

class CoordinateExcelStylesheet implements Stylesheet {
    public static final String STYLE_HEADER = "header";

    @Override
    void declareStyles(CanDefineStyle stylable) {
        stylable.style(STYLE_HEADER, st -> {
            st.font(f -> f.style(FontStyle.BOLD))
//            st.align(Keywords.VerticalAlignment.CENTER, Keywords.HorizontalAlignment.CENTER)
        })
    }
}