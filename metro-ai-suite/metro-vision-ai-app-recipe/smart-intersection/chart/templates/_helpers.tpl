{{- define "requiredFile" -}}
{{- $root := index . 0 -}}
{{- $path := index . 1 -}}
{{- required (printf "Missing required file: %s" $path) ($root.Files.Get $path) -}}
{{- end -}}

{{- define "secrets-py" -}}
SECRET_KEY='{{ randAlphaNum 50 }}'
DATABASE_PASSWORD='{{ .Values.pgserver.password }}'
{{- end -}}

{{- define "supass" -}}
{{ required "You must set supass (e.g. --set supass=...) for this chart to install." .Values.supass }}
{{- end -}}

{{- define "db-password" -}}
{{ required "You must set pgserver.password (e.g. --set pgserver.password=...) for this chart to install." .Values.pgserver.password }}
{{- end -}}